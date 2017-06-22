#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>
#include <math.h>
#include <fstream>
#include <vector>
#include <numeric>
#include <algorithm>
#include <functional>
#include <bitset>
#include <cstring>
#include <sstream>
#include <thread>
#include <random>
#include <condition_variable>

#include <wiringPi.h>
#include "yaml-cpp/yaml.h"
#include "docopt/docopt.h"
#include "MQTTClient.h"
extern "C" {
    #include "cbuffer/cbuffer.h"
}

using namespace std::chrono;
// using namespace moodycamel;

std::string app_name = "rfmqtt";
std::string app_version = "0.1";

int pin = -1;

bool raw = false;
bool scan = false;

float dev_start_max = 0.2;
float dev_trit_max = 0.2;
float dev_mean_target = 0.05;
float quality_min = 0.5;
int hold_ms = 1000;
int press_ms = 800;

// BlockingReaderWriterQueue<std::pair<int, int>> queue(1000);
// BlockingReaderWriterQueue<int> queue(1000);
cbuf_t *cbuffer = cbuf_new(14);

high_resolution_clock::time_point last_time;



#define MCHECK(call) { \
    int rc = call; \
    if (rc != MQTTCLIENT_SUCCESS) { \
        std::cerr << "Call to " #call " failed (" << rc << ")" << std::endl; \
    } \
}


int mqtt_received(void *context, char *topicName, int topicLen, MQTTClient_message *message);
void mqtt_delivered(void *context, MQTTClient_deliveryToken dt);
void mqtt_disconnected(void *context, char *cause);

class Mqtt {

    MQTTClient client = nullptr;
    MQTTClient_connectOptions opts = MQTTClient_connectOptions_initializer;
    
public:
    std::string client_id;

    std::string broker;
    std::string username;
    std::string password;
    int keep_alive_interval = -1;
    int qos = 0;

    int backoff_min = 50;
    int backoff_max = 2000;
    int backoff_cur;

    Mqtt() {
        backoff_cur = backoff_min;
    }

    void connect() {
        opts.username = username.c_str();
        opts.password = password.c_str();
        if (keep_alive_interval != -1) {
            opts.keepAliveInterval = keep_alive_interval;
        }
        opts.cleansession = true;

        MCHECK(MQTTClient_create(&client, broker.c_str(), client_id.c_str(), MQTTCLIENT_PERSISTENCE_NONE, nullptr));
        MCHECK(MQTTClient_setCallbacks(client, this, mqtt_disconnected, mqtt_received, mqtt_delivered));
        int status = MQTTClient_connect(client, &opts);
        if (status == MQTTCLIENT_SUCCESS) {
            backoff_cur = backoff_min;
            std::cout << "Connected" << std::endl;
        } else {
            reconnect();
        }
    }

    void reconnect() {
        std::cout << "Reconnecting in " << backoff_cur << " ms" << std::endl;
        std::this_thread::sleep_for(milliseconds(backoff_cur));
        backoff_cur += std::rand() % backoff_cur;
        backoff_cur = std::min(backoff_cur, backoff_max);
        connect();
    }

    void publish(std::string topic, std::string payload) {
        printf("< %s %s\n", topic.c_str(), payload.c_str());
        MCHECK(MQTTClient_publish(
            client,
            topic.c_str(),
            payload.size(),
            &payload[0],
            qos,
            true,
            nullptr
        ));
    }

    int received(char *topicName, int topicLen, MQTTClient_message *message)
    {
        std::string topic;

        if (topicLen > 0) {
            topic.resize(topicLen);
            memcpy(&topic[0], topicName, topicLen);
        }
        else {
            topic = topicName;
        }

        std::string payload;
        payload.resize(message->payloadlen);
        memcpy(&payload[0], message->payload, message->payloadlen);
        printf("> %s: %s\n", topic.c_str(), payload.c_str());
        
        MQTTClient_freeMessage(&message);
        MQTTClient_free(topicName);
        return 1;
    }

    void delivered(MQTTClient_deliveryToken dt)
    {
        fprintf(stderr, "Delivered: %d\n", dt);
    }

    void disconnected(char *cause)
    {
        fprintf(stderr, "Disconnected\n");
        reconnect();
    }

} mqtt;

int mqtt_received(void *context, char *topicName, int topicLen, MQTTClient_message *message) {
    auto queue = static_cast<Mqtt*>(context);
    return queue->received(topicName, topicLen, message);
}

void mqtt_delivered(void *context, MQTTClient_deliveryToken dt)
{
    auto queue = static_cast<Mqtt*>(context);
    queue->delivered(dt);
}

void mqtt_disconnected(void *context, char *cause)
{
    auto queue = static_cast<Mqtt*>(context);
    queue->disconnected(cause);
}



float getPatternDeviation(std::vector<int> pattern, std::vector<int> micros) {
    float pattern_sum = std::accumulate(pattern.begin(), pattern.end(), 0);
    float micros_sum = std::accumulate(micros.begin(), micros.begin() + pattern.size(), 0);

    std::vector<float > diffsq(pattern.size());
    
    std::transform(pattern.begin(), pattern.end(), micros.begin(), diffsq.begin(), [pattern_sum, micros_sum](int p, int m) {
        float diff = (p / pattern_sum) - (m / micros_sum);
        return diff * diff;
    });

    return sqrt(std::accumulate(diffsq.begin(), diffsq.end(), 0.0) / pattern.size());
}

std::pair<int, float> getBestPattern(std::vector<std::vector<int>> patterns, std::vector<int> micros) {
    int min_index = -1;
    float min_dev = 1;
    for (int i = 0; i < (int)patterns.size(); i++) {
        auto pattern = patterns[i];
        float dev = getPatternDeviation(pattern, micros);
        if (dev < min_dev) {
            min_dev = dev;
            min_index = i;
        }
    }
    return { min_index, min_dev };
}

struct Topic {
    std::string name;
    int group = -1;
    int unit = -1;
    int onoff = -1;

    high_resolution_clock::time_point last_time;
    int prev_state = -1;
};

class RfDevice {
    std::thread holder;

public:
    uint32_t address;
    std::bitset<31> address_bits;
    std::vector<Topic> topics;
    bool running = true;
    bool delaying = false;

    std::mutex mutex;
    std::condition_variable cv;
    high_resolution_clock::time_point last_update;
    high_resolution_clock::time_point wake_point;

    RfDevice() {}
    RfDevice(const RfDevice& device) :
        holder(),

        address(device.address),
        address_bits(device.address_bits),
        topics(device.topics),
        running(device.running) {}

    ~RfDevice() {
        running = false;
        if (holder.joinable()) holder.join();
    }

    void init() {
        address_bits = address;
        holder = std::thread(&RfDevice::hold_runner, this);
    }

    void hold_runner() {
        while (running) {
            
            {
                std::unique_lock<std::mutex> lock(mutex);
                cv.wait(lock, [this] { return this->delaying; });
                delaying = false;

                high_resolution_clock::time_point next_wake = wake_point;
                lock.unlock();

                std::this_thread::sleep_until(next_wake);
                high_resolution_clock::time_point awoken;
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    awoken = wake_point;
                    if (next_wake == awoken) {
                        for (auto &topic : topics) {
                            if (topic.onoff == -1) continue;
                            if (topic.prev_state == 1) {
                                mqtt.publish(topic.name, "OFF");
                                topic.prev_state = 0;
                            }
                        }
                    }
                }

            }

            // finished = true;
            // std::this_thread::sleep_for(milliseconds(500)); 
            // std::cout << "finished" << std::endl;


        } 
    }

    void update(int group, int unit, int onoff, int dim) {
        
        high_resolution_clock::time_point update_time = high_resolution_clock::now();
        // printf("%7d %d %d %d %d\n", address, group, unit, onoff, dim);

        for (auto &topic : topics) {
            if (topic.group != -1 && topic.group != group) continue;
            if (topic.unit != -1 && topic.unit != unit) continue;
            if (topic.onoff != -1) {
                if (topic.onoff != onoff) continue;
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    wake_point = update_time + milliseconds(press_ms);
                    delaying = true;
                }
                cv.notify_all();
                if (topic.prev_state != 1) {
                    mqtt.publish(topic.name, "ON");
                    topic.prev_state = 1;
                }
            } else {
                if (topic.prev_state == onoff) {
                    auto millis = duration_cast<milliseconds>(update_time - topic.last_time);
                    if (millis.count() < hold_ms) {
                        continue;
                    }
                }
                std::string onoff_str = onoff == -1 ? "NA" : onoff ? "ON" : "OFF";
                mqtt.publish(topic.name, onoff_str);
                topic.prev_state = onoff;
                // printf("< %s %s\n", topic.name.c_str(), onoff_str.c_str());
            }

            topic.last_time = update_time;

        }

        last_update = update_time;

    }

};

std::vector<RfDevice> devices;

namespace YAML {

    template<>
    struct convert<Topic> {
        static Node encode(const Topic& rhs) {
            Node node;
            return node;
        }
        static bool decode(const Node& node, Topic& topic) {
            topic.name = node["name"].as<std::string>();
            if (node["group"]) topic.group = node["group"].as<int>();
            if (node["unit"]) topic.unit = node["unit"].as<int>();
            if (node["onoff"]) topic.onoff = node["onoff"].as<int>();
            return true;
        }
    };

    template<>
    struct convert<RfDevice> {
        static Node encode(const RfDevice& rhs) {
            Node node;
            return node;
        }
        static bool decode(const Node& node, RfDevice& device) {
            if (node.Type() != YAML::NodeType::Map) {
                return false;
            }
            device.address = node["address"].as<int>();
            
            auto topics = node["topics"];
            for (std::size_t i = 0; i < topics.size(); i++) {
                Topic topic = topics[i].as<Topic>();
                device.topics.push_back(topic);
            }

            return true;
        }
    };
}

struct RfDetector {

    static const int micro_num = 4;

    std::vector<std::vector<int>> start_patterns = {
        {1, 3},
        {1, 6}
    };

    std::vector<std::vector<int>> trit_patterns = {
        {1, 1, 1, 5},
        {1, 5, 1, 1},
        {1, 1, 1, 1}
    };

    std::vector<std::vector<int>> end_patterns = {
        {1, 40},
        {1, 650}
    };

    std::pair<int, float> start_pulse;
    std::pair<int, float> end_pulse;

    int id = -1;

    bool valid = true;
    bool finished = false;

    int edge_state = 1;
    std::vector<int> local_micros = std::vector<int>(micro_num);
    int local_edge = 0;
    int trits_end = 32;
    int trit_num = 0;
    std::vector<std::pair<int, float>> trits;
    int edge_num = 0;

    uint32_t address = -1;
    int group;
    int onoff;
    int unit;
    int dim = -1;

    uint32_t extractBits(int from, int to) {
        std::bitset<32> bits;
        int num = to - from;
        for (int i = 0; i < num; i++) {
            auto trit = trits[to - 1 - i];
            bits[i] = trit.first;
        }
        return bits.to_ulong();
    }

    void advance(int micro, int state) {

        if (finished) return;

        edge_num++;
        
        if (!valid) return;

        // if (id >= 0) {
            // printf("%4d %d %d trit %d, local edge %d, %4d us\n",
                // id, state, edge_state, trit_num, local_edge, micro);
        // }
        
        local_micros[local_edge] = micro;
        
        if (edge_num == 1) {
            if (state != 1) valid = false;
        }
        if (edge_num == 2) {
            if (state != 0) valid = false;
            start_pulse = getBestPattern(start_patterns, local_micros);
            local_edge = -1;
            if (start_pulse.second > dev_start_max) {
                valid = false;
            }
        } else if (edge_num > 2 && edge_state != state) {
            valid = false;
        }

        if (trit_num >= trits_end && local_edge >= 1) {

            end_pulse = getBestPattern(end_patterns, local_micros);
            
            auto devs = std::vector<float>(trits.size());
            std::transform(trits.begin(), trits.end(), devs.begin(), [](std::pair<int, float> trit) {
                return trit.second;
            });

            float dev_mean = (
                start_pulse.second +
                end_pulse.second +
                std::accumulate(devs.begin(), devs.end(), 0.0)
            )  / (trits.size() + 2);

            float dev_min = std::min({
                start_pulse.second,
                end_pulse.second,
                *std::min_element(devs.begin(), devs.end())
            });

            float dev_max = std::max({
                start_pulse.second,
                end_pulse.second,
                *std::max_element(devs.begin(), devs.end())
            });

            float quality = 1/(1 + dev_mean/dev_mean_target);

            if (quality > quality_min) {
                if (scan) {
                    printf("%6d %3.f%% address %07x  group %d  onoff %d  unit %2d  dim %2d   all %0.3f  min %0.3f  max %0.3f\n",
                        id, quality*100, address, group, onoff, unit, dim, dev_mean, dev_min, dev_max
                    );
                } else {
                    // std::cout << end_pulse.first << " " << end_pulse.second << std::endl;
                    for (auto &device : devices) {
                        if (device.address == address) {
                            device.update(group, unit, onoff, dim);
                        }
                    }
                }
                
            }

            finished = true;
        } else if (local_edge < 3) {
            local_edge++;
        } else {
            auto trit = getBestPattern(trit_patterns, local_micros);

            if (trit.second > dev_trit_max) {
                valid = false;
            }

            trits.push_back(trit);
            trit_num++;
            // std::cout << id << " trit value " << trit.first << "   " << trit.second << "   trits " << trit_num << std::endl;

            if (!scan && trit_num <= 26) {
                bool some = false;
                for (auto &device : devices) {
                    if (trit.first == device.address_bits[26 - trit_num]) some = true;
                }
                if (!some) valid = false;
            }

            switch (trit_num) {
                case 26:
                    address = extractBits(0, 26);
                    // printf("address %08x\n", address);
                    break;

                case 27:
                    group = trits[26].first;
                    // printf("group %d\n", group);
                    break;

                case 28:
                    onoff = trits[27].first;
                    // printf("onoff %d\n", onoff);
                    if (onoff == 2) {
                        trits_end = 36;
                    }
                    break;

                case 32:
                    unit = extractBits(28, 32);
                    // printf("unit %d\n", unit);
                    break;

                case 36:
                    dim = extractBits(32, 36);
                    // printf("dim %dx\n", dim);
                    break;
            }

            local_edge = 0;
        }

        edge_state = !edge_state;
    }

};

std::vector<RfDetector> detectors;


class RfStream {

    int id = 0;
    int last_micro = 0;
    int last_state = -1;
    
public:
    void advance(int micro, int state) {
        
        if (raw) {
            printf("%d\t%d\n", micro, state);
            return;
        }
        
        for (auto &detector : detectors) {
            detector.advance(micro, state);
        }

        detectors.erase(std::remove_if(
            detectors.begin(), detectors.end(),
            [](const RfDetector& detector) { 
                return !detector.valid || detector.finished;
            }
        ), detectors.end());


        RfDetector detector;
        detector.advance(last_micro, last_state);
        detector.advance(micro, state);
        if (detector.valid) {
            detector.id = id++;
            detectors.push_back(detector);
        }

        last_state = state;
        last_micro = micro;
    }
} stream;


void interrupt() {
    // Negate state as the timing applies to the previous state, before the edge
    int microstate[2];
    int state = 1 - digitalRead(pin);
    high_resolution_clock::time_point read_time = high_resolution_clock::now();
    auto micro = duration_cast<microseconds>(read_time - last_time);
    int m = micro.count();
    microstate[0] = m;
    microstate[1] = state;

    cbuf_offer(cbuffer, (unsigned char*)&microstate[0], 8);
    // queue.try_enqueue({ m, state });
    // queue.try_enqueue(m);
    last_time = read_time;
}


static const char USAGE[] =
R"(rfmqtt

    Usage:
        rfmqtt [--config=<file>] [--file=<file>]
        rfmqtt raw [--config=<file>] [--file=<file>]
        rfmqtt scan [--config=<file>] [--file=<file>]
        rfmqtt file <raw.tsv>
        rfmqtt (-h | --help)
        rfmqtt --version

    Options:
        --config=<file> Use a different configuration YAML file.
        -f --file=<file> Read raw values from a file.
        -h --help     Show this screen.
        --version     Show version.
)";

std::string getEnvVar(std::string const& key)
{
    char const* val = getenv(key.c_str()); 
    return val == NULL ? std::string() : std::string(val);
}

std::string getEnvPath(std::string const& key, std::string const& path)
{
    std::string val = getEnvVar(key);
    if (val.empty()) return val;
    return val + path;
}

int main (int argc, const char** argv)
{
    std::map<std::string, docopt::value> args
        = docopt::docopt(USAGE,
                         { argv + 1, argv + argc },
                         true,           // show help if requested
                         app_name + " " + app_version);  // version string
 
    std::string config_name = "config.yaml";
    std::vector<std::string> config_paths = {
        "./" + config_name,
        getEnvPath("XDG_CONFIG_HOME", "/" + app_name + "/" + config_name),
        getEnvPath("HOME", "/.config/" + app_name + "/" + config_name),
        getEnvPath("XDG_CONFIG_DIRS", "/" + app_name + "/" + config_name),
        "/etc/xdg/" + app_name + "/" + config_name,
    };
    
    if (args["--config"]) {
        config_paths.insert(config_paths.begin(), args["--config"].asString());
    }
    
    mqtt.client_id = app_name;

    try {
        YAML::Node config;

        bool config_loaded = false;
        std::string config_paths_tried;
        for (auto config_path : config_paths) {
            if (config_path.empty()) continue;
            try {
                config = YAML::LoadFile(config_path);
            } catch (YAML::BadFile badFile) {
                config_paths_tried += "  " + config_path + " (" + badFile.msg + ")\n";
                continue;
            }
            config_loaded = true;
            std::cout << "Config: " << config_path << std::endl;
            break;
        }
        if (!config_loaded) {
            throw std::string("config file not found at any of the following paths:\n") + config_paths_tried;
        }

        pin = config["pin"].as<int>();

        auto broker = config["broker"];
        auto username = config["username"];
        auto password = config["password"];
        auto keepalive = config["keepalive"];
        
        if (!broker) throw "'broker' not found";
        mqtt.broker = broker.as<std::string>();
        std::cout << "Broker: " << mqtt.broker << std::endl;

        if (username) {
            mqtt.username = username.as<std::string>();
            std::cout << "Username: " << mqtt.username << std::endl;
        }
        if (password) {
            mqtt.password = password.as<std::string>();
            std::cout << "Password: *****" << std::endl;
        }
        if (keepalive) {
            mqtt.keep_alive_interval = keepalive.as<int>();
            std::cout << "Keep alive: " << mqtt.keep_alive_interval << std::endl;
        }

        auto config_devices = config["devices"];
        std::cout << "Topics:" << std::endl;
        for (unsigned int i = 0; i < config_devices.size(); i++) {
            RfDevice device = config_devices[i].as<RfDevice>();
            for (auto &topic : device.topics) {
                printf("- %s ( %7x / %d / %d )\n",
                    topic.name.c_str(), device.address, topic.group, topic.unit);
            }
            devices.push_back(device);
            RfDevice &dev = devices.back();
            dev.init();
        }
        
        auto params = config["params"];
        if (params) {
            std::cout << "Params:" << std::endl;
            if (params["start-max"]) {
                dev_start_max = params["start-max"].as<float>();
                std::cout << "  start-max: " << dev_start_max << std::endl;
            }
            if (params["trit-max"]) {
                dev_trit_max = params["trit-max"].as<float>();
                std::cout << "  trit-max: " << dev_trit_max << std::endl;
            }
            if (params["mean-target"]) {
                dev_mean_target = params["mean-target"].as<float>();
                std::cout << "  mean-target: " << dev_mean_target << std::endl;
            }
            if (params["quality-min"]) {
                quality_min = params["quality-min"].as<float>();
                std::cout << "  quality-min: " << quality_min << std::endl;
            }
            if (params["hold-ms"]) {
                hold_ms = params["hold-ms"].as<int>();
                std::cout << "  hold-ms: " << hold_ms << std::endl;
            }
            if (params["press-ms"]) {
                hold_ms = params["press-ms"].as<int>();
                std::cout << "  press-ms: " << hold_ms << std::endl;
            }
        }

        std::cout << std::endl;

        mqtt.connect();
    
    } catch (std::string msg) {
        std::cerr << "Error: " << msg << std::endl;
        return 1;
    } catch (const char* msg) {
        std::cerr << "Error: " << msg << std::endl;
        return 1;
    }

    // Print arguments
    // for(auto const& arg : args) {
        // std::cout << arg.first <<  arg.second << std::endl;
    // }

    if (args["raw"].asBool()) raw = true;
    if (args["scan"].asBool()) scan = true;

    if (args["--file"]) {
        std::string file = args["--file"].asString();
        std::ifstream recording(file);
        int line = 0;
        int micro, state;
        while (recording >> micro >> state)
        {
            // if (line >= 5725 && line < 5725 + 132*2) stream.advance(micro, state);
            stream.advance(micro, state);
            line++;
        }
    } else {
        wiringPiSetupGpio();

        wiringPiISR(pin, INT_EDGE_BOTH, interrupt);
        
        // pinMode(pin, INPUT);
        // int last_state = -1;
        // int count = 0;
        // int poll_interval = 50;
        for (;;)
        {
            // std::this_thread::sleep_for(microseconds(poll_interval));
            // int state = digitalRead(pin);
            // count++;
            // if (last_state != state) {
            //     std::cout << poll_interval*count << "\t" << state << std::endl;
            //     last_state = state;
            //     count = 0;
            // }

            std::this_thread::sleep_for(milliseconds(100));
            int used = cbuf_usedspace(cbuffer);
            
            // std::cout << used << std::endl;

            const int int_size = sizeof(int);
            int ints = used / int_size;
            int* p = (int*)cbuf_peek(cbuffer);
            for (int i = 0; i < ints; i += 2) {
                int micro = p[i];
                int state = p[i + 1];
                stream.advance(micro, state);
            }
            cbuf_poll(cbuffer, ints*int_size);


            // std::pair<int, int> microstate;
            // queue.wait_dequeue(microstate);
            // stream.advance(microstate.first, microstate.second);
            // int micro;
            // queue.wait_dequeue(micro);
        }
    }

    return 0;
}
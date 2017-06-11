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

#include <wiringPi.h>
#include "yaml-cpp/yaml.h"
#include "docopt/docopt.h"
#include "readerwriterqueue/readerwriterqueue.h"
#include "MQTTClient.h"

using namespace std::chrono;
using namespace moodycamel;

std::string app_name = "rfmqtt";

int pin = 17;

bool raw = false;
bool scan = false;

float dev_start_max = 0.1;
float dev_trit_max = 0.1;
float dev_mean_target = 0.05;
float quality_min = 0.7;
int hold_ms = 250;

BlockingReaderWriterQueue<std::pair<int, int>> queue(1000);

high_resolution_clock::time_point last_time;
// microseconds last_micro;
// microseconds period_micro;
// int last_state;

/*
int edge_num = 0;
int edge_state;
int period;
unsigned int address;

void readPeriodBit(int period, int state) {
    std::cout << period << ": " << state << std::endl;
}

void interrupt() {
    // Negate state as the timing applies to the previous state, before the edge
    int state = 1 - digitalRead(pin);
    high_resolution_clock::time_point read_time = high_resolution_clock::now();
    auto micro = duration_cast<microseconds>(read_time - last_time);
    
    std::cout << micro.count() << " " << state << std::endl;

    // auto lm = last_micro.count();
    auto m = micro.count();
    // std::cout << rm << std::endl;

    if (edge_num < 2) {
        // if (last_state == 1 && state == 0 && lm*8 < m && lm*12 > m) {
        //     period_micro = last_micro;
        //     auto rm = (double)m / lm;
        //     std::cout << "Possible start pulse " << rm << " " << lm << "us " << m/10 << "us" << std::endl;
        //     // edge_num = 2;
        //     edge_state = state;

        //     address = 0;
        //     period = 0;
        // }
    }
    if (edge_num >= 2) {
        if (edge_state != state) {
            std::cout << "Invalid edge transition, aborting" << std::endl;
            edge_num = 0;
        } else {
            edge_num++;
            int batch = round((float)m / period_micro.count());
            std::cout << "batch " << batch << std::endl;
            // while (batch > 0) {
            //     readPeriodBit(period, state);
            //     period++;
            //     batch--;
            // }
        }
    }
    edge_state = !edge_state;

    last_time = read_time;
    last_micro = micro;
    last_state = state;
}
*/

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
    std::string prefix;
    int keep_alive_interval = -1;
    int qos = 0;

    Mqtt() {

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
        MCHECK(MQTTClient_connect(client, &opts));
    }

    void publish(std::string topic, std::string payload) {
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
        fprintf(stderr, "Connection lost: %s\n", cause);
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
    fprintf(stderr, "\nConnection lost: %s\n", cause);
}



float getPatternDeviation(std::vector<int> pattern, std::vector<int> micros) {
    float pattern_sum = std::accumulate(pattern.begin(), pattern.end(), 0);
    float micros_sum = std::accumulate(micros.begin(), micros.begin() + pattern.size(), 0);

    std::vector<float > diffsq(pattern.size());
    
    // std::transform(pattern.begin(), pattern.end(), micros.begin(), std::back_inserter(diffsq), [pattern_sum, micros_sum](int p, int m) {
        // return (p / pattern_sum) - (m / micros_sum);
        // return 5;
    // });

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

struct RfDevice {
    uint32_t address;
    std::bitset<31> address_bits;

    std::vector<std::vector<std::string>> topics;

    high_resolution_clock::time_point last_time;

    void init() {
        address_bits = address;
    }

    void add_topic(int group, int unit, std::string topic) {
        if ((unsigned int)group >= topics.size()) {
            topics.resize(group + 1);
        }

        auto &topics_unit = topics[group];
        if ((unsigned int)unit >= topics_unit.size()) {
            topics_unit.resize(unit + 1, "");
        }

        topics_unit[unit] = topic;
    }

    const std::string get_topic(int group, int unit) {
        if ((unsigned int)group >= topics.size()) {
            return "";
        }
        
        auto &topics_unit = topics[group];
        if ((unsigned int)unit >= topics_unit.size()) {
            return "";
        }

        return topics[group][unit];
    } 

    void update(int group, int unit, int onoff, int dim) {
        
        // int prev_onoff = state_unit[unit];
        // state_unit[unit] = onoff;
    
        high_resolution_clock::time_point update_time = high_resolution_clock::now();
        auto millis = duration_cast<milliseconds>(update_time - last_time);

        if (millis.count() > hold_ms) {
            // std::string prev_onoff_str = prev_onoff == -1 ? "NA" : prev_onoff ? "ON" : "OFF";
            std::string onoff_str = onoff == -1 ? "NA" : onoff ? "ON" : "OFF";

            std::ostringstream topic;
            topic << "switch/";
            topic << address;
            topic << "/";
            topic << group;
            topic << "/";
            topic << unit;

            mqtt.publish(topic.str(), onoff_str);

            printf("> %07x / %d / %d / %s\n", address, group, unit, onoff_str.c_str());
        }

        last_time = update_time;

    }

};

std::vector<RfDevice> devices;

namespace YAML {
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

            std::cout << "topics " << topics.size() << std::endl;

            for (auto topic = topics.begin(); topic != topics.end(); topic++) {
                std::cout << topic->second << std::endl;
                // std::cout << topic["name"] << std::endl;
                // std::cout << topic["group"] << std::endl;
                // std::cout << topic["unit"] << std::endl;
            }
            
            // device.add_topic()
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

        if (id >= 0) {
            // printf("%4d %d %d trit %d, local edge %d, %4d us\n",
                // id, state, edge_state, trit_num, local_edge, micro);
        }
        
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
                }
                
                for (auto &device : devices) {
                    if (device.address == address) {
                        device.update(group, unit, onoff, dim);
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
    int state = 1 - digitalRead(pin);
    high_resolution_clock::time_point read_time = high_resolution_clock::now();
    auto micro = duration_cast<microseconds>(read_time - last_time);
    int m = micro.count();
    queue.try_enqueue({ m, state });
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
      -f --file=<file> Read raw values from a file.
      -h --help     Show this screen.
      --version     Show version.
)";


int main (int argc, const char** argv)
{

    std::string config_file = "config.yaml";

    std::map<std::string, docopt::value> args
        = docopt::docopt(USAGE,
                         { argv + 1, argv + argc },
                         true,           // show help if requested
                         "rfmqtt 0.1");  // version string

    if (args["--config"]) config_file = args["--config"].asString();
    
    mqtt.client_id = app_name;

    try {
        YAML::Node config = YAML::LoadFile(config_file);

        auto config_devices = config["devices"];

        for (unsigned int i = 0; i < config_devices.size(); i++) {
            RfDevice device = config_devices[i].as<RfDevice>();
            device.init();
            // printf("%d %7x\n", i, device.address);
            devices.push_back(device);
        }

        auto broker = config["broker"];
        auto username = config["username"];
        auto password = config["password"];
        auto keepalive = config["keepalive"];
        auto prefix = config["prefix"];

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
        if (prefix) {
            mqtt.prefix = prefix.as<std::string>();
        }

        std::cout << std::endl;

        mqtt.connect();

    } catch (YAML::BadFile badFile) {
        std::cerr << "Error: " << config_file << " not found (" << badFile.msg << ")" << std::endl;
        return 1;
    } catch (const char* msg) {
        std::cerr << "Error: " << msg << std::endl;
        return 2;
    }

    // for(auto const& arg : args) {
        // std::cout << arg.first <<  arg.second << std::endl;
    // }

    if (args["raw"].asBool()) raw = true;
    if (args["scan"].asBool()) scan = true;

    if (args["--file"]) {
        std::string file = args["--file"].asString();
        std::ifstream recording(file);
        // std::cout << "Replaying from " << file << std::endl;
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
        for (;;)
        {
            // int v = digitalRead(pin);
            // high_resolution_clock::time_point read_time = high_resolution_clock::now();

            // duration<double> elapsed = duration_cast<duration<double>>(read_time - start);

            // printf("%f %d\n", elapsed.count(), v);
            
            std::pair<int, int> microstate;
            queue.wait_dequeue(microstate);

            stream.advance(microstate.first, microstate.second);
            
            // if (queue.size_approx() > 0) std::cout << queue.size_approx() << " " << microstate.first << " " << microstate.second << std::endl;
        }
    }

    return 0;
}
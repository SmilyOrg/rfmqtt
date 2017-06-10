#include <iostream>
#include <wiringPi.h>
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

using namespace std::chrono;

int pin = 17;

float dev_start_max = 0.1;
float dev_trit_max = 0.1;
float dev_mean_target = 0.05;
float quality_min = 0.7;

high_resolution_clock::time_point last_time;
microseconds last_micro;
microseconds period_micro;
int last_state;

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
                printf("%6d %3.f%% address %07x  group %d  onoff %d  unit %2d  dim %2d   all %0.3f  min %0.3f  max %0.3f\n",
                    id, quality*100, address, group, onoff, unit, dim, dev_mean, dev_min, dev_max
                );
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
        // std::cout << "" << state << "   " << micro << "us " << std::endl;
        
        int valid = 0;

        for (auto &detector : detectors) {
            detector.advance(micro, state);
            if (detector.valid) valid++;
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
};



int main (void)
{
    std::ifstream recording("../allfour-rf.csv");

    RfStream stream;

    int line = 0;
    
    int micro, state;

    while (recording >> micro >> state)
    {
        // if (line >= 5725 && line < 5725 + 132*2) stream.advance(micro, state);
        stream.advance(micro, state);
        line++;
    }

    return 0;

    /*
    last_time = high_resolution_clock::now();

    wiringPiSetupGpio();
    // pinMode(pin, INPUT);

    wiringPiISR(pin, INT_EDGE_BOTH, interrupt);

    for (;;)
    {
        // int v = digitalRead(pin);
        // high_resolution_clock::time_point read_time = high_resolution_clock::now();

        // duration<double> elapsed = duration_cast<duration<double>>(read_time - start);

        // printf("%f %d\n", elapsed.count(), v);
        delay(1000);
    }
    return 0;
    */
}
#pragma once

#include <random>
#include <utility>
#include <vector>
#include "nlohmann/json.hpp"

struct DelayConfig
{
    enum class EventModel
    {
        Bernoulli,
        Poisson
    };

    enum class DurationModel
    {
        Uniform,
        Gaussian
    };

    unsigned int seed = 0;
    int minDelay = 1;
    int maxDelay = 1;
    EventModel eventModel = EventModel::Bernoulli;
    double pDelay = 0.0;
    DurationModel durationModel = DurationModel::Uniform;
    double gaussMeanRatio = 0.5;
    double gaussStdRatio = 0.0;
};

class DelayGenerator
{
public:
    DelayGenerator(const DelayConfig& config, int num_of_agents);

    std::vector<std::pair<int, int>> nextTick();
    void clear_active_delays();

    const DelayConfig& get_config() const { return config; }
    const std::vector<int>& get_remaining_delays() const { return remaining_delay; }
    int get_current_tick() const { return current_tick; }
    const std::vector<std::vector<std::pair<int, int>>>& get_delay_intervals() const { return delay_intervals; }
    nlohmann::ordered_json delay_intervals_to_json() const;

private:
    void validate_config() const;
    int sample_delay_duration();
    std::vector<int> collect_available_agents() const;
    std::vector<int> sample_agents_for_delay(const std::vector<int>& available_agents);

    DelayConfig config;
    int num_of_agents = 0;
    int current_tick = 0;
    std::vector<int> remaining_delay;
    std::vector<std::vector<std::pair<int, int>>> delay_intervals;
    std::mt19937 rng;
};

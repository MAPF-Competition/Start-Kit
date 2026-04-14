#include "DelayGenerator.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

DelayGenerator::DelayGenerator(const DelayConfig& config, int num_of_agents):
    config(config),
    num_of_agents(num_of_agents),
    remaining_delay(num_of_agents, 0),
    delay_intervals(num_of_agents),
    rng(config.seed)
{
    if (num_of_agents < 0)
    {
        throw std::invalid_argument("DelayGenerator: number of agents must be non-negative");
    }
    validate_config();
}

void DelayGenerator::validate_config() const
{
    if (config.minDelay <= 0)
    {
        throw std::invalid_argument("DelayConfig.minDelay must be > 0");
    }
    if (config.maxDelay < config.minDelay)
    {
        throw std::invalid_argument("DelayConfig.maxDelay must be >= minDelay");
    }
    if (config.pDelay < 0.0 || config.pDelay > 1.0)
    {
        throw std::invalid_argument("DelayConfig.pDelay must be in [0, 1]");
    }
    if (config.durationModel == DelayConfig::DurationModel::Gaussian)
    {
        if (config.gaussMeanRatio < 0.0 || config.gaussMeanRatio > 1.0)
        {
            throw std::invalid_argument("DelayConfig.gaussMeanRatio must be in [0, 1] for gaussian");
        }
        if (config.gaussStdRatio < 0.0)
        {
            throw std::invalid_argument("DelayConfig.gaussStdRatio must be >= 0 for gaussian");
        }
    }
}

std::vector<int> DelayGenerator::collect_available_agents() const
{
    std::vector<int> available_agents;
    available_agents.reserve(num_of_agents);
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        if (remaining_delay[agent] == 0)
        {
            available_agents.push_back(agent);
        }
    }
    return available_agents;
}

int DelayGenerator::sample_delay_duration()
{
    if (config.minDelay == config.maxDelay)
    {
        return config.minDelay;
    }

    if (config.durationModel == DelayConfig::DurationModel::Uniform)
    {
        std::uniform_int_distribution<int> dist(config.minDelay, config.maxDelay);
        return dist(rng);
    }

    const double span = static_cast<double>(config.maxDelay - config.minDelay);
    const double mean = static_cast<double>(config.minDelay) + span * config.gaussMeanRatio;
    const double stdev = span * config.gaussStdRatio;
    if (stdev <= 0.0)
    {
        return std::clamp(static_cast<int>(std::lround(mean)), config.minDelay, config.maxDelay);
    }

    std::normal_distribution<double> dist(mean, stdev);
    return std::clamp(static_cast<int>(std::lround(dist(rng))), config.minDelay, config.maxDelay);
}

std::vector<int> DelayGenerator::sample_agents_for_delay(const std::vector<int>& available_agents)
{
    std::vector<int> selected_agents;
    if (available_agents.empty())
    {
        return selected_agents;
    }

    if (config.eventModel == DelayConfig::EventModel::Bernoulli)
    {
        std::bernoulli_distribution dist(config.pDelay);
        for (int agent : available_agents)
        {
            if (dist(rng))
            {
                selected_agents.push_back(agent);
            }
        }
        return selected_agents;
    }

    const double poisson_lambda = static_cast<double>(num_of_agents) * config.pDelay;
    std::poisson_distribution<int> count_dist(poisson_lambda);
    const int sampled_count = count_dist(rng);
    const int selected_count = std::min(sampled_count, static_cast<int>(available_agents.size()));
    if (selected_count <= 0)
    {
        return selected_agents;
    }

    std::vector<int> shuffled_agents = available_agents;
    std::shuffle(shuffled_agents.begin(), shuffled_agents.end(), rng);
    shuffled_agents.resize(selected_count);
    return shuffled_agents;
}

std::vector<std::pair<int, int>> DelayGenerator::nextTick()
{
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        if (remaining_delay[agent] > 0)
        {
            remaining_delay[agent]--;
        }
    }

    const std::vector<int> available_agents = collect_available_agents();
    const std::vector<int> selected_agents = sample_agents_for_delay(available_agents);

    std::vector<std::pair<int, int>> events;
    events.reserve(selected_agents.size());
    for (int agent : selected_agents)
    {
        const int duration = sample_delay_duration();
        remaining_delay[agent] = duration;
        delay_intervals[agent].push_back({current_tick, current_tick + duration});
        events.push_back({agent, duration});
    }

    current_tick++;
    return events;
}

void DelayGenerator::clear_active_delays()
{
    std::fill(remaining_delay.begin(), remaining_delay.end(), 0);
}

nlohmann::ordered_json DelayGenerator::delay_intervals_to_json() const
{
    nlohmann::ordered_json result = nlohmann::ordered_json::array();
    for (const auto& agent_intervals : delay_intervals)
    {
        nlohmann::ordered_json agent_json = nlohmann::ordered_json::array();
        for (const auto& interval : agent_intervals)
        {
            agent_json.push_back({interval.first, interval.second});
        }
        result.push_back(agent_json);
    }
    return result;
}

#include <cassert>
#include <iostream>
#include <vector>

#include "DelayGenerator.h"

int main()
{
    {
        DelayConfig config;
        config.seed = 12345;
        config.minDelay = 1;
        config.maxDelay = 3;
        config.eventModel = DelayConfig::EventModel::Poisson;
        config.pDelay = 0.2;
        config.durationModel = DelayConfig::DurationModel::Gaussian;
        config.gaussMeanRatio = 0.5;
        config.gaussStdRatio = 0.22;

        DelayGenerator lhs(config, 10);
        DelayGenerator rhs(config, 10);

        for (int tick = 0; tick < 100; tick++)
        {
            const auto lhs_events = lhs.nextTick();
            const auto rhs_events = rhs.nextTick();
            assert(lhs_events == rhs_events);
        }
    }

    {
        DelayConfig config;
        config.seed = 0;
        config.minDelay = 2;
        config.maxDelay = 2;
        config.eventModel = DelayConfig::EventModel::Bernoulli;
        config.pDelay = 1.0;
        config.durationModel = DelayConfig::DurationModel::Uniform;
        config.gaussMeanRatio = 0.5;
        config.gaussStdRatio = 0.0;

        DelayGenerator generator(config, 1);
        const auto tick0 = generator.nextTick();
        const auto tick1 = generator.nextTick();
        const auto tick2 = generator.nextTick();

        assert(tick0.size() == 1);
        assert(tick0[0] == std::make_pair(0, 2));
        assert(tick1.empty());
        assert(tick2.size() == 1);
        assert(tick2[0] == std::make_pair(0, 2));
    }

    std::cout << "delay_generator test passed" << std::endl;
    return 0;
}

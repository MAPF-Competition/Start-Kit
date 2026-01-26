#pragma once
#include "common.h"

struct Delay
{
    int minDelay;
    int maxDelay;
    bool currentDelay;
    int delayCounter;
    Delay(): minDelay(-1), maxDelay(-1), currentDelay(false), delayCounter(0) {}
    void tick()
    {
        delayCounter++;
        if (currentDelay && delayCounter > maxDelay)
        {
            currentDelay = false;
            delayCounter = 0;
        }
    }

    bool inDelay() const
    {
        return currentDelay;
    }

    bool operator==(const Delay& other) const{
        return minDelay == other.minDelay && maxDelay == other.maxDelay && currentDelay == other.currentDelay && delayCounter == other.delayCounter;
    }

    bool operator!=(const Delay& other) const{
        return !(*this == other);
    }
};

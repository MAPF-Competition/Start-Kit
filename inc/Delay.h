#pragma once
#include "common.h"

struct Delay
{
    int minDelay;
    int maxDelay;
    //TODO: Do we know the exact delay time?
    int currentDelay;
    int delayCounter;
    Delay(): minDelay(-1), maxDelay(-1), currentDelay(-1), delayCounter(0) {}
    void tick()
    {
        delayCounter++;
        if (currentDelay != -1 && delayCounter > currentDelay)
        {
            currentDelay = -1;
            delayCounter = 0;
        }
    }

    bool inDelay() const
    {
        return currentDelay != -1;
    }

    bool operator==(const Delay& other) const{
        return minDelay == other.minDelay && maxDelay == other.maxDelay && currentDelay == other.currentDelay && delayCounter == other.delayCounter;
    }

    bool operator!=(const Delay& other) const{
        return !(*this == other);
    }
};

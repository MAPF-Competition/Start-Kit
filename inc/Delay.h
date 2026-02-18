#pragma once
#include "common.h"

struct Delay
{
    int minDelay;
    int maxDelay;
    bool inDelay;
    Delay(): minDelay(-1), maxDelay(-1), inDelay(false) {}

    bool operator==(const Delay& other) const{
        return minDelay == other.minDelay && maxDelay == other.maxDelay && inDelay == other.inDelay;
    }

    bool operator!=(const Delay& other) const{
        return !(*this == other);
    }
};

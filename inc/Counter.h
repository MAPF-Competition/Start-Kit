#pragma once
#include "common.h"

struct Counter
{
    int count;
    int maxCount;
    Counter(): count(0), maxCount(0) {}
    bool tick()
    {
        // Returns true if reaches maxCount and resets. The agent moves to the next location when this function returns true. Otherwise the agent is on the edge.
        if (maxCount <= 0){
            throw std::runtime_error("Error: Invalid maxCount value = " + std::to_string(maxCount));
        }
        count++;
        if (count > maxCount)
        {
            count = 0;
            return true;
        }
        return false;
    }
    bool operator==(const Counter& other) const{
        return count == other.count && maxCount == other.maxCount;
    }
    bool operator!=(const Counter& other) const{
        return !(*this == other);
    }
};
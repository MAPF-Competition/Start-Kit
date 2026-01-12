#pragma once
#include "common.h"
#include "Counter.h"
#include "Delay.h"


struct State
{
    int location;
    int timestep;
    int orientation;  // 0:east, 1:south, 2:west, 3:north
    Counter counter;
    Delay delay;

    struct Hasher
    {
        size_t operator()(const State& n) const
        {
            size_t loc_hash = std::hash<int>()(n.location);
            size_t time_hash = std::hash<int>()(n.timestep);
            size_t ori_hash = std::hash<int>()(n.orientation);
            //TODO: Hash the counter too?
            return (time_hash ^ (loc_hash << 1) ^ (ori_hash << 2));
        }
    };

    void operator = (const State& other)
    {
        timestep = other.timestep;
        location = other.location;
        orientation = other.orientation;
        counter = other.counter;
        delay = other.delay;
    }

    bool operator == (const State& other) const
    {
        return timestep == other.timestep && location == other.location && orientation == other.orientation && counter.count == other.counter.count && delay.currentDelay == other.delay.currentDelay;
    }

    bool operator != (const State& other) const
    {
        return timestep != other.timestep || location != other.location || orientation != other.orientation;
    }

    State(): location(-1), timestep(-1), orientation(-1), counter(Counter()), delay(Delay()) {}
    // State(int loc): loc(loc), timestep(0), orientation(0) {}
    // State(int loc, int timestep): loc(loc), timestep(timestep), orientation(0) {}
    State(int location, int timestep = -1, int orientation = -1, Counter counter = Counter(), Delay delay = Delay()):
        location(location), timestep(timestep), orientation(orientation) {}
    State(const State& other):
        location(other.location), timestep(other.timestep), orientation(other.orientation), counter(other.counter), delay(other.delay) {}
};

std::ostream & operator << (std::ostream &out, const State &s);

typedef std::vector<State> Path;

std::ostream & operator << (std::ostream &out, const Path &path);

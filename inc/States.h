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
    Action current_action;

    struct Hasher
    {
        size_t operator()(const State& n) const
        {
            size_t loc_hash = std::hash<int>()(n.location);
            size_t time_hash = std::hash<int>()(n.timestep);
            size_t ori_hash = std::hash<int>()(n.orientation);
            size_t counter_hash = std::hash<int>()(n.counter.count);
            size_t delay_hash = std::hash<bool>()(n.delay.inDelay);
            size_t action_hash = std::hash<int>()(static_cast<int>(n.current_action));
            return (time_hash ^ (loc_hash << 1) ^ (ori_hash << 2) ^ (counter_hash << 3) ^ (delay_hash << 4) ^ (action_hash << 5));
        }
    };

    void operator = (const State& other)
    {
        timestep = other.timestep;
        location = other.location;
        orientation = other.orientation;
        counter = other.counter;
        delay = other.delay;
        current_action = other.current_action;
    }

    bool operator == (const State& other) const
    {
        return timestep == other.timestep && location == other.location && orientation == other.orientation && counter.count == other.counter.count && delay.inDelay == other.delay.inDelay && current_action == other.current_action;
    }

    bool operator != (const State& other) const
    {
        return timestep != other.timestep || location != other.location || orientation != other.orientation || counter.count != other.counter.count || delay.inDelay != other.delay.inDelay || current_action != other.current_action;
    }

    State(): location(-1), timestep(-1), orientation(-1), counter(Counter()), delay(Delay()), current_action(Action::NA) {}

    State(int location, int timestep = -1, int orientation = -1, int max_counter = 10, Delay delay = Delay(), Action current_action = Action::NA):
        location(location), timestep(timestep), orientation(orientation), counter(Counter(max_counter)), delay(delay), current_action(current_action) {}
    State(const State& other):
        location(other.location), timestep(other.timestep), orientation(other.orientation), counter(other.counter), delay(other.delay), current_action(other.current_action) {}
};

inline std::ostream & operator << (std::ostream &out, const State &s)
{
    out << "loc=" << s.location
        << ",ori=" << s.orientation
        << ",t=" << s.timestep
        << ",counter=" << s.counter.count << "/" << s.counter.maxCount
        << ",delay={min=" << s.delay.minDelay
        << ",max=" << s.delay.maxDelay
        << ",in=" << s.delay.inDelay << "}";
    return out;
}

typedef std::vector<State> Path;

inline std::ostream & operator << (std::ostream &out, const Path &path)
{
    for (auto state : path)
    {
        if (state.location < 0)
            continue;
        out << "(" << state << ")->";
    }
    out << std::endl;
    return out;
}

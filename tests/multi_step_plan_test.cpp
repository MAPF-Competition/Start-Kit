#include <cassert>
#include <iostream>
#include <vector>

#include "planner.h"
#include "utils.h"

namespace {

State apply_action_simple(const State& s, Action a, const SharedEnvironment* env)
{
    State next = s;
    next.timestep = s.timestep + 1;

    if (a == Action::CR)
    {
        next.orientation = (s.orientation + 1) % 4;
        return next;
    }
    if (a == Action::CCR)
    {
        next.orientation = (s.orientation + 3) % 4;
        return next;
    }
    if (a == Action::W)
    {
        return next;
    }

    int delta = 0;
    if (s.orientation == 0) delta = 1;
    if (s.orientation == 1) delta = env->cols;
    if (s.orientation == 2) delta = -1;
    if (s.orientation == 3) delta = -env->cols;

    const int nxt = s.location + delta;
    if (nxt >= 0 && nxt < static_cast<int>(env->map.size()) && DefaultPlanner::validateMove(s.location, nxt, env))
    {
        next.location = nxt;
    }
    return next;
}

} // namespace

int main()
{
    SharedEnvironment env;
    env.rows = 5;
    env.cols = 5;
    env.num_of_agents = 3;
    env.map.assign(env.rows * env.cols, 0);
    env.curr_timestep = 0;

    env.curr_states = {
        State(0, 0, 0),   // top-left, facing east
        State(4, 0, 2),   // top-right, facing west
        State(20, 0, 0)   // bottom-left, facing east
    };

    env.goal_locations.resize(env.num_of_agents);
    env.goal_locations[0].push_back({24, 0});
    env.goal_locations[1].push_back({20, 0});
    env.goal_locations[2].push_back({4, 0});

    DefaultPlanner::initialize(1000, &env);

    std::vector<std::vector<Action>> multi_actions;
    const int num_steps = 6;
    DefaultPlanner::plan(200, multi_actions, &env, num_steps);

    assert(static_cast<int>(multi_actions.size()) == env.num_of_agents);
    for (int i = 0; i < env.num_of_agents; i++)
    {
        assert(static_cast<int>(multi_actions[i].size()) == num_steps);
    }

    // Roll out locally and validate basic action consistency.
    std::vector<State> sim = env.curr_states;
    for (int t = 0; t < num_steps; t++)
    {
        std::vector<State> next(sim.size());
        for (int a = 0; a < env.num_of_agents; a++)
        {
            Action act = multi_actions[a][t];
            assert(act == Action::FW || act == Action::CR || act == Action::CCR || act == Action::W);
            next[a] = apply_action_simple(sim[a], act, &env);
        }

        // no vertex collision in this simple test setup
        for (int i = 0; i < env.num_of_agents; i++)
        {
            for (int j = i + 1; j < env.num_of_agents; j++)
            {
                assert(next[i].location != next[j].location);
            }
        }
        sim.swap(next);
    }

    std::cout << "multi_step_plan test passed" << std::endl;
    return 0;
}

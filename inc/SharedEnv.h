#pragma once
#include "States.h"
#include "Grid.h"
#include "nlohmann/json.hpp"


class SharedEnvironment
{
public:
    int num_of_agents;
    int rows;
    int cols;
    std::string map_name;
    std::vector<int> map;
    std::string file_storage_path;

    // goal locations for each agent
    // each task is a pair of <goal_loc, reveal_time>
    vector< vector<pair<int, int> > > goal_locations;

    int curr_timestep = 0;
    vector<State> curr_states;

    SharedEnvironment(){}
};

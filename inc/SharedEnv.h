#pragma once
#include "States.h"
#include "Grid.h"
#include "nlohmann/json.hpp"
#include "Tasks.h"


typedef std::chrono::steady_clock::time_point TimePoint;
typedef std::chrono::milliseconds milliseconds;

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

    vector<Task> task_pool;
    // unordered_map<int,Task> task_pool;
    vector<int> curr_task_schedule;

    // plan_start_time records the time point that plan/initialise() function is called; 
    // It is a convenient variable to help planners/schedulers to keep track of time.
    // plan_start_time is updated when the simulation system call the entry plan function, its type is std::chrono::steady_clock::time_point
    TimePoint plan_start_time;

    SharedEnvironment(){}
};

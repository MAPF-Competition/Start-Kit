#pragma once
#include "States.h"
#include "Grid.h"
#include "nlohmann/json.hpp"


class SharedEnvironment {
public:
    int num_of_agents;

    int rows;
    int cols;
    std::vector<int> map;


    // goal locations for each agent
    // each task is a pair of <goal_loc, reveal_time>
    vector< vector<pair<int, int> > > goal_locations;

    int curr_timestep = 0;
    vector<State> curr_states;

    SharedEnvironment(){}



    //initialize the environment using inputJSON
    // SharedEnvironment(std::string inputJSON){
    //     using json=nlohmann::json;
    //     std::ifstream file(inputJSON);
    //     json j;
    //     file>>j;
    //     std::string map_file=j["map_file"];
    //     std::string agent_file=j["agent_file"];
    //     std::string task_file=j["task_file"];
    //     int  num_tasks_reveal=j["num_tasks_reveal"];
    //     Grid grid(map_file);
    //     std::vector<int> start_locs = read_int_vec(agent_file);
	//     std::vector<int> tasks = read_int_vec(task_file);
    //     this->rows=grid.rows;
    //     this->cols=grid.cols;
    //     this->map=grid.map;
        

    //     this->num_of_agents=start_locs.size();
    //     this->goal_locations.resize(num_of_agents);




    // }

};

/**
 * @file Simulator.h
 * @brief header file for Simulator.cpp.
 * 
 * @authors Zhe Chen Han Zhang, Yue Zhang
 * 
 * @note All authors contributed equally to this work.
 */

#include "ActionModel.h"
#include "SharedEnv.h"
#include "States.h"
#include "nlohmann/json.hpp"


class Simulator
{
public:
    Simulator(Grid &grid, std::vector<int>& start_locs, ActionModelWithRotate* model):
        map(grid), model(model)
    {
        num_of_agents = start_locs.size();
        starts.resize(num_of_agents);
        paths.resize(num_of_agents);

        for (size_t i = 0; i < start_locs.size(); i++)
            {
                if (grid.map[start_locs[i]] == 1)
                {
                    cout<<"error: agent "<<i<<"'s start location is an obstacle("<<start_locs[i]<<")"<<endl;
                    exit(0);
                }
                starts[i] = State(start_locs[i], 0, 0);
        }

        curr_states = starts;

        actual_movements.resize(num_of_agents);
        planner_movements.resize(num_of_agents);
    }

    vector<State> move(vector<Action>& next_actions);

    //void sync_shared_env(SharedEnvironment* env);

    vector<State> get_current_state(){ return curr_states; }

    int get_curr_timestep() {return timestep;}

    bool get_all_valid(){ return all_valid;}

    void sync_shared_env(SharedEnvironment* env);

    nlohmann::ordered_json actual_path_to_json() const;

    nlohmann::ordered_json planned_path_to_json() const;

    nlohmann::ordered_json starts_to_json() const;

    nlohmann::ordered_json action_errors_to_json() const;

    int get_number_errors() const {return model->errors.size();}

private:
    Grid map;

    ActionModelWithRotate* model;


    // #timesteps for simulation
    int timestep = 0;

    std::vector<Path> paths;

    vector<State> starts;
    int num_of_agents;

    vector<State> curr_states;

    vector<list<Action>> actual_movements;
    vector<list<Action>> planner_movements;

    bool all_valid = true;
    
};
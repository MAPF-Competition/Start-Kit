#pragma once


#include"SharedEnv.h"



#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include <pybind11/numpy.h>



// in case of someone wants to implement a gym wrapper, 
// they can modify the pyEnvironment and expose them to python

class pyEnvironment{
public:

    pyEnvironment(SharedEnvironment *env):env(env){}



    std::vector<State> get_curr_states(){
        return env->curr_states;
    }

    State get_agent_curr_state(int agent){
        return env->curr_states[agent];
    }


    // pybind11::array_t<int> get_goal_locations();
    pybind11::array_t<int> get_map(){
        pybind11::array_t<int> map_array(env->map.size());
        auto map_data = map_array.mutable_data();
        for (auto i = 0; i < env->map.size(); i++) {
            map_data[i] = env->map[i];
        }
        return map_array;
    }
    int get_rows(){return env->rows;}
    int get_cols(){return env->cols;}
    int get_num_of_agents(){return env->num_of_agents;}

    std::string get_file_storage_path(){
        return env->file_storage_path;
    }
    
    // this will make a copy and will be less efficient if called frequently
    std::vector<std::vector<std::pair<int,int>>>get_goal_locations(){
        return env->goal_locations;
    }

    std::vector<std::pair<int,int>> get_agent_goal_locations(int agent){
        return env->goal_locations[agent];
    }

    std::string get_map_name(){ return env->map_name; }
    int get_currtimestep(){return env->curr_timestep;}

    SharedEnvironment* env;
};
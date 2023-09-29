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


    pybind11::array_t<int> get_curr_states(){
        int num_agents=env->curr_states.size();
        pybind11::array_t<int,pybind11::array::c_style> states_array({num_agents,3});
        int * states_data=states_array.mutable_data();
        int* ptr = states_data;
        for (auto i = 0; i < env->map.size(); i++) {
            ptr[3 * i] = env->curr_states[i].location;
            ptr[3 * i + 1] = env->curr_states[i].orientation;
            ptr[3 * i + 2] = env->curr_states[i].timestep;
        }  
        return states_array;
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
    
    std::vector<std::vector<std::pair<int,int>>>get_goal_locations(){
        return env->goal_locations;
    }

    std::string get_map_name(){ return env->map_name; }
    int get_currtimestep(){return env->curr_timestep;}

    SharedEnvironment* env;
};
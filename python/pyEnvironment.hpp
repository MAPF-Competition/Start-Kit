#pragma once


#include"SharedEnv.h"



#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include <pybind11/numpy.h>




class pyEnvironment{
public:

    pyEnvironment(SharedEnvironment *env):env(env){}

    pybind11::array_t<int> get_curr_states(){
        int num_agents=env->curr_states.size();
        pybind11::array_t<int,pybind11::array::c_style> states_array({num_agents,3});
        auto states_data=states_array.mutable_unchecked();
        for(auto i=0;i<env->map.size();i++){
            states_data(i,0)=env->curr_states[i].location;
            states_data(i,1)=env->curr_states[i].orientation;
            states_data(i,2)=env->curr_states[i].timestep;
            
        }
        return states_array;
    }
    // pybind11::array_t<int> get_goal_locations();
    pybind11::array_t<int> get_map(){
        pybind11::array_t<int> map_array(env->map.size());
        auto map_data=map_array.mutable_unchecked<1>();
        for(auto i=0;i<env->map.size();i++){
            map_data[i]=env->map[i];
        }
        return map_array;
    }
    int get_rows(){return env->rows;}
    int get_cols(){return env->cols;}
    int get_num_of_agents(){return env->num_of_agents;}
    
    std::vector<std::vector<std::pair<int,int>>>get_goal_locations(){
        return env->goal_locations;
    }
    int get_currtimestep(){return env->curr_timestep;}
private:
    SharedEnvironment* env;
};
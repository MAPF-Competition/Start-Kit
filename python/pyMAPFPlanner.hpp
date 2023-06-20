#pragma once




#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include <pybind11/numpy.h>
#include "SharedEnv.h"
#include "States.h"
#include "MAPFPlanner.h"
#include "pyEnvironment.hpp"



class pyMAPFPlanner:public MAPFPlanner{
public:
    pyMAPFPlanner();
    ~pyMAPFPlanner(){
        // delete py_env;
    }


    void initialize(int preprocess_time_limit);
    // std::vector<Action> plan(int time_limit);
    void plan(int time_limit,std::vector<Action>&plan);
    


private:
    pybind11::object py_planner;
    pyEnvironment * py_env;
};



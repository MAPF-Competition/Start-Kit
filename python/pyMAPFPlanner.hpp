#pragma once




#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include <pybind11/numpy.h>
#include "SharedEnv.h"
#include "States.h"



class pyMAPFPlanner{
public:
    pyMAPFPlanner();
    ~pyMAPFPlanner();

    SharedEnvironment* env;
    void initialize(int preprocess_time_limit);
    std::vector<State> plan(int time_limit);



private:
    pybind11::object py_planner;



};



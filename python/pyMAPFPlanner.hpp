#pragma once




#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include <pybind11/numpy.h>
#include "SharedEnv.h"
#include "States.h"
#include "MAPFPlanner.h"



class pyMAPFPlanner:public MAPFPlanner{
public:
    pyMAPFPlanner();
    // ~pyMAPFPlanner();


    void initialize(int preprocess_time_limit);
    std::vector<Action> plan(int time_limit);



private:
    pybind11::object py_planner;



};



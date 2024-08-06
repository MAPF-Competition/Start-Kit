#pragma once
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include <pybind11/numpy.h>
#include "SharedEnv.h"
#include "States.h"
#include "TaskScheduler.h"

class pyTaskScheduler: public TaskScheduler
{
private:
    pybind11::object py_scheduler;
    /* data */
public:
    pyTaskScheduler(SharedEnvironment* env);


    void initialize(int preprocess_time_limit);
    void plan(int time_limit, std::vector<int> & proposed_schedule);
};



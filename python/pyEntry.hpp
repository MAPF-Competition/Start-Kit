#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include <pybind11/numpy.h>
#include "SharedEnv.h"
#include "States.h"
#include "Entry.h"
#include "pyMAPFPlanner.hpp"
#include "pyTaskScheduler.hpp"

class PyEntry : public Entry
{
public:
    PyEntry(SharedEnvironment *env) : Entry(env){
        planner=new pyMAPFPlanner(env);
        scheduler= new pyTaskScheduler(env);
    }
    PyEntry()
    {
        env = new SharedEnvironment();
        planner = new pyMAPFPlanner(env);
        scheduler = new pyTaskScheduler(env);
    }
};
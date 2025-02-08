/**
 * @file pyTaskScheduler.hpp
 * @brief header file for pyTaskScheduler
 *  
 * @authors Teng Guo
 * 
 * @note All authors contributed equally to this work.
 */
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

public:
    pyTaskScheduler(SharedEnvironment* env);

};



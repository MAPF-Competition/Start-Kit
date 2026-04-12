/**
 * pyTaskScheduler.hpp
 * C++ bridge: TaskScheduler subclass that delegates to a Python implementation.
 */
#pragma once
#include "TaskScheduler.h"
#include <pybind11/pybind11.h>
#include <pybind11/embed.h>

namespace py = pybind11;

class pyTaskScheduler : public TaskScheduler
{
public:
    pyTaskScheduler(SharedEnvironment* env) : TaskScheduler(env) {}
    pyTaskScheduler() : TaskScheduler() {}
    ~pyTaskScheduler() override = default;

    void load_python_scheduler();

    void initialize(int preprocess_time_limit) override;
    void plan(int time_limit, std::vector<int> & proposed_schedule) override;

private:
    py::object py_scheduler;
    bool loaded = false;
};

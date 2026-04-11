/**
 * pyMAPFPlanner.hpp
 * C++ bridge: MAPFPlanner subclass that delegates to a Python implementation.
 */
#pragma once
#include "MAPFPlanner.h"
#include <pybind11/pybind11.h>
#include <pybind11/embed.h>

namespace py = pybind11;

class pyMAPFPlanner : public MAPFPlanner
{
public:
    pyMAPFPlanner(SharedEnvironment* env) : MAPFPlanner(env) {}
    pyMAPFPlanner() : MAPFPlanner() {}
    ~pyMAPFPlanner() override = default;

    // Load and instantiate the user's Python planner
    void load_python_planner();

    void initialize(int preprocess_time_limit) override;
    void plan(int time_limit, Plan & plan) override;

private:
    py::object py_planner;
    bool loaded = false;
};

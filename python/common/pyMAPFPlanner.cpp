/**
 * pyMAPFPlanner.cpp
 * C++ bridge implementation for Python MAPFPlanner.
 */
#include "pyMAPFPlanner.hpp"
#include "opaque_types.h"

namespace py = pybind11;

void pyMAPFPlanner::load_python_planner()
{
    if (loaded) return;
    try {
        py::module_ sys = py::module_::import("sys");
        // Add user_planner directory to path
        sys.attr("path").attr("insert")(0, "python/user_planner");
        py::module_ mod = py::module_::import("pyMAPFPlanner");
        py_planner = mod.attr("pyMAPFPlanner")();
        loaded = true;
    } catch (py::error_already_set &e) {
        std::cerr << "Failed to load Python MAPFPlanner: " << e.what() << std::endl;
        throw;
    }
}

void pyMAPFPlanner::initialize(int preprocess_time_limit)
{
    py::gil_scoped_acquire acquire;
    // Ensure MAPF module type bindings are registered before any py::cast
    py::module_::import("MAPF");
    load_python_planner();
    // Pass env to Python planner
    py_planner.attr("env") = py::cast(env, py::return_value_policy::reference);
    py_planner.attr("initialize")(preprocess_time_limit);
}

void pyMAPFPlanner::plan(int time_limit, Plan & plan)
{
    py::gil_scoped_acquire acquire;
    py::list py_actions = py_planner.attr("plan")(time_limit);

    // Convert Python list of lists of Action (int) to C++ vector<vector<Action>>
    plan.actions.clear();
    for (auto agent_actions : py_actions) {
        std::vector<Action> actions_vec;
        for (auto act : agent_actions) {
            actions_vec.push_back(static_cast<Action>(act.cast<int>()));
        }
        plan.actions.push_back(std::move(actions_vec));
    }
}

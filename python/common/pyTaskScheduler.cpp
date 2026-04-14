/**
 * pyTaskScheduler.cpp
 * C++ bridge implementation for Python TaskScheduler.
 */
#include "pyTaskScheduler.hpp"
#include "opaque_types.h"

namespace py = pybind11;

void pyTaskScheduler::load_python_scheduler()
{
    if (loaded) return;
    try {
        py::module_ sys = py::module_::import("sys");
        sys.attr("path").attr("insert")(0, "python/user_scheduler");
        py::module_ mod = py::module_::import("pyTaskScheduler");
        py_scheduler = mod.attr("pyTaskScheduler")();
        loaded = true;
    } catch (py::error_already_set &e) {
        std::cerr << "Failed to load Python TaskScheduler: " << e.what() << std::endl;
        throw;
    }
}

void pyTaskScheduler::initialize(int preprocess_time_limit)
{
    py::gil_scoped_acquire acquire;
    // Ensure MAPF module type bindings are registered before any py::cast
    py::module_::import("MAPF");
    load_python_scheduler();
    py_scheduler.attr("env") = py::cast(env, py::return_value_policy::reference);
    py_scheduler.attr("initialize")(preprocess_time_limit);
}

void pyTaskScheduler::plan(int time_limit, std::vector<int> & proposed_schedule)
{
    py::gil_scoped_acquire acquire;
    py::list result = py_scheduler.attr("plan")(time_limit);
    proposed_schedule.clear();
    for (auto item : result) {
        proposed_schedule.push_back(item.cast<int>());
    }
}

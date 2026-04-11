/**
 * pyExecutor.cpp
 * C++ bridge implementation for Python Executor.
 */
#include "pyExecutor.hpp"
#include "opaque_types.h"

namespace py = pybind11;

void pyExecutor::load_python_executor()
{
    if (loaded) return;
    try {
        py::module_ sys = py::module_::import("sys");
        sys.attr("path").attr("insert")(0, "python/user_executor");
        py::module_ mod = py::module_::import("pyExecutor");
        py_executor = mod.attr("pyExecutor")();
        loaded = true;
    } catch (py::error_already_set &e) {
        std::cerr << "Failed to load Python Executor: " << e.what() << std::endl;
        throw;
    }
}

void pyExecutor::initialize(int preprocess_time_limit)
{
    py::gil_scoped_acquire acquire;
    // Ensure MAPF module type bindings are registered before any py::cast
    py::module_::import("MAPF");
    load_python_executor();
    py_executor.attr("env") = py::cast(env, py::return_value_policy::reference);
    py_executor.attr("initialize")(preprocess_time_limit);
}

vector<State> pyExecutor::process_new_plan(int sync_time_limit, Plan & plan, vector<vector<Action>> & staged_actions)
{
    py::gil_scoped_acquire acquire;
    // Pass plan by reference (opaque type, no copy).
    // Pass staged_actions by reference — Python modifies in-place via the opaque VectorVectorAction wrapper.
    py::object result = py_executor.attr("process_new_plan")(
        sync_time_limit,
        py::cast(plan, py::return_value_policy::reference),
        py::cast(staged_actions, py::return_value_policy::reference)
    );

    // Python returns predicted_states (list of State).
    // Since State is a registered type, we can iterate and cast.
    std::vector<State> predicted;
    for (auto s : result) {
        predicted.push_back(s.cast<State>());
    }
    return predicted;
}

void pyExecutor::next_command(int exec_time_limit, std::vector<ExecutionCommand> & agent_command)
{
    py::gil_scoped_acquire acquire;
    py::object result = py_executor.attr("next_command")(exec_time_limit);
    agent_command.clear();
    for (auto cmd : result) {
        agent_command.push_back(static_cast<ExecutionCommand>(cmd.cast<int>()));
    }
}

/**
 * pyExecutor.hpp
 * C++ bridge: Executor subclass that delegates to a Python implementation.
 */
#pragma once
#include "Executor.h"
#include <pybind11/pybind11.h>
#include <pybind11/embed.h>

namespace py = pybind11;

class pyExecutor : public Executor
{
public:
    pyExecutor(SharedEnvironment* env) : Executor(env) {}
    pyExecutor() : Executor() {}
    ~pyExecutor() override = default;

    void load_python_executor();

    void initialize(int preprocess_time_limit) override;
    vector<State> process_new_plan(int sync_time_limit, Plan & plan, vector<vector<Action>> & staged_actions) override;
    void next_command(int exec_time_limit, std::vector<ExecutionCommand> & agent_command) override;

private:
    py::object py_executor;
    bool loaded = false;
};

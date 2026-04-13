/**
 * Pybind11 bindings for the MAPF competition types.
 * 
 * IMPORTANT: All container types are bound as opaque (zero-copy) wrappers.
 * Python code gets a reference to the C++ container — no data is copied on access.
 * This is critical for performance with large maps and many agents.
 * 
 * Example: env.map[i] reads directly from the C++ vector, not a Python list copy.
 */
#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>
#include <pybind11/operators.h>
#ifndef MAPF_SHARED_MODULE
#include <pybind11/embed.h>
#endif

#include "SharedEnv.h"
#include "ActionModel.h"
#include "Plan.h"
#include "Tasks.h"
#include "States.h"
#include "Counter.h"
#include "Delay.h"
#include "opaque_types.h"

namespace py = pybind11;

// Module init function — shared between .so and embedded builds
static void init_mapf_module(py::module_& m) {
    m.doc() = "MAPF Competition Python Bindings (zero-copy container access)";

    // =====================================================================
    // Bind container types as opaque wrappers (zero-copy, reference semantics)
    // =====================================================================

    // vector<int> — used for map, new_tasks, new_freeagents, curr_task_schedule, Task.locations
    py::bind_vector<std::vector<int>>(m, "VectorInt", py::buffer_protocol())
        .def("__repr__", [](const std::vector<int> &v) {
            std::string r = "VectorInt[";
            for (size_t i = 0; i < std::min(v.size(), (size_t)5); i++) {
                if (i) r += ", ";
                r += std::to_string(v[i]);
            }
            if (v.size() > 5) r += ", ...";
            r += "] (size=" + std::to_string(v.size()) + ")";
            return r;
        });

    // vector<State> — used for curr_states, system_states, start_states
    py::bind_vector<std::vector<State>>(m, "VectorState")
        .def("__repr__", [](const std::vector<State> &v) {
            return "VectorState(size=" + std::to_string(v.size()) + ")";
        });

    // vector<Action> — inner vector for Plan.actions and staged_actions
    py::bind_vector<std::vector<Action>>(m, "VectorAction")
        .def("__repr__", [](const std::vector<Action> &v) {
            return "VectorAction(size=" + std::to_string(v.size()) + ")";
        });

    // vector<vector<Action>> — Plan.actions and staged_actions
    py::bind_vector<std::vector<std::vector<Action>>>(m, "VectorVectorAction")
        .def("__repr__", [](const std::vector<std::vector<Action>> &v) {
            return "VectorVectorAction(size=" + std::to_string(v.size()) + ")";
        });

    // vector<pair<int,int>> — inner vector for goal_locations
    py::bind_vector<std::vector<std::pair<int, int>>>(m, "VectorPairInt")
        .def("__repr__", [](const std::vector<std::pair<int, int>> &v) {
            return "VectorPairInt(size=" + std::to_string(v.size()) + ")";
        });

    // vector<vector<pair<int,int>>> — goal_locations
    py::bind_vector<std::vector<std::vector<std::pair<int, int>>>>(m, "VectorVectorPairInt")
        .def("__repr__", [](const std::vector<std::vector<std::pair<int, int>>> &v) {
            return "VectorVectorPairInt(size=" + std::to_string(v.size()) + ")";
        });

    // unordered_map<int, Task> — task_pool
    py::bind_map<std::unordered_map<int, Task>>(m, "TaskPool")
        .def("__repr__", [](const std::unordered_map<int, Task> &m) {
            return "TaskPool(size=" + std::to_string(m.size()) + ")";
        });

    // pair<int,int> — used in goal_locations
    // pair<int,int> — read-only access via .first/.second using __getattr__
    // pybind11 3.0's method_adaptor doesn't support member lambdas on std::pair,
    // so we bind it without methods and add accessors via module-level helpers.
    py::class_<std::pair<int, int>>(m, "PairInt")
        .def(py::init<>())
        .def(py::init([](int a, int b) { return std::pair<int,int>(a, b); }))
        .def("__repr__", [](const std::pair<int, int> &p) {
            return "(" + std::to_string(p.first) + ", " + std::to_string(p.second) + ")";
        });

    // Module-level helpers for pair access
    m.def("pair_first", [](const std::pair<int, int> &p) { return p.first; });
    m.def("pair_second", [](const std::pair<int, int> &p) { return p.second; });

    // =====================================================================
    // Enums
    // =====================================================================

    py::enum_<Action>(m, "Action")
        .value("FW", Action::FW)
        .value("CR", Action::CR)
        .value("CCR", Action::CCR)
        .value("W", Action::W)
        .value("NA", Action::NA)
        .export_values();

    py::enum_<ExecutionCommand>(m, "ExecutionCommand")
        .value("GO", ExecutionCommand::GO)
        .value("STOP", ExecutionCommand::STOP)
        .export_values();

    // =====================================================================
    // Structs
    // =====================================================================

    py::class_<Counter>(m, "Counter")
        .def(py::init<int>(), py::arg("max_counter") = 10)
        .def_readwrite("count", &Counter::count)
        .def_readwrite("maxCount", &Counter::maxCount)
        .def("tick", &Counter::tick);

    py::class_<Delay>(m, "Delay")
        .def(py::init<>())
        .def_readwrite("minDelay", &Delay::minDelay)
        .def_readwrite("maxDelay", &Delay::maxDelay)
        .def_readwrite("inDelay", &Delay::inDelay);

    py::class_<State>(m, "State")
        .def(py::init<>())
        .def(py::init<int, int, int>(), py::arg("location"), py::arg("timestep") = -1, py::arg("orientation") = -1)
        .def_readwrite("location", &State::location)
        .def_readwrite("timestep", &State::timestep)
        .def_readwrite("orientation", &State::orientation)
        .def_readwrite("counter", &State::counter)
        .def_readwrite("delay", &State::delay)
        .def("__repr__", [](const State &s) {
            return "State(loc=" + std::to_string(s.location)
                + ", ori=" + std::to_string(s.orientation)
                + ", t=" + std::to_string(s.timestep) + ")";
        });

    py::class_<Task>(m, "Task")
        .def(py::init<>())
        .def_readwrite("task_id", &Task::task_id)
        .def_readwrite("t_completed", &Task::t_completed)
        .def_readwrite("t_revealed", &Task::t_revealed)
        .def_readwrite("agent_assigned", &Task::agent_assigned)
        .def_readwrite("locations", &Task::locations)
        .def_readwrite("idx_next_loc", &Task::idx_next_loc)
        .def("get_next_loc", &Task::get_next_loc)
        .def("is_finished", &Task::is_finished);

    py::class_<Plan>(m, "Plan")
        .def(py::init<>())
        .def_readwrite("actions", &Plan::actions);

    // =====================================================================
    // SharedEnvironment
    // =====================================================================

    py::class_<SharedEnvironment>(m, "SharedEnvironment")
        .def(py::init<>())
        .def_readwrite("num_of_agents", &SharedEnvironment::num_of_agents)
        .def_readwrite("rows", &SharedEnvironment::rows)
        .def_readwrite("cols", &SharedEnvironment::cols)
        .def_readwrite("map_name", &SharedEnvironment::map_name)
        .def_readwrite("map", &SharedEnvironment::map)
        .def_readwrite("file_storage_path", &SharedEnvironment::file_storage_path)
        .def_readwrite("goal_locations", &SharedEnvironment::goal_locations)
        .def_readwrite("curr_timestep", &SharedEnvironment::curr_timestep)
        .def_readwrite("system_timestep", &SharedEnvironment::system_timestep)
        .def_readwrite("curr_states", &SharedEnvironment::curr_states)
        .def_readwrite("start_states", &SharedEnvironment::start_states)
        .def_readwrite("system_states", &SharedEnvironment::system_states)
        .def_readwrite("task_pool", &SharedEnvironment::task_pool)
        .def_readwrite("new_tasks", &SharedEnvironment::new_tasks)
        .def_readwrite("new_freeagents", &SharedEnvironment::new_freeagents)
        .def_readwrite("curr_task_schedule", &SharedEnvironment::curr_task_schedule)
        .def_readwrite("staged_actions", &SharedEnvironment::staged_actions)
        .def_readwrite("min_planner_communication_time", &SharedEnvironment::min_planner_communication_time)
        .def_readwrite("action_time", &SharedEnvironment::action_time)
        .def_readwrite("max_counter", &SharedEnvironment::max_counter);
}

// MAPF_SHARED_MODULE: build as loadable .so for standalone Python use
// Otherwise: register as embedded module for the C++ executable's interpreter
#ifdef MAPF_SHARED_MODULE
PYBIND11_MODULE(MAPF, m) { init_mapf_module(m); }
#else
PYBIND11_EMBEDDED_MODULE(MAPF, m) { init_mapf_module(m); }
#endif

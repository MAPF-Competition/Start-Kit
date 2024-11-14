#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include <pybind11/numpy.h>
#include <pybind11/chrono.h>

#include "Grid.h"
#include "SharedEnv.h"
#include "MAPFPlanner.h"
#include "States.h"
#include "pyEnvironment.hpp"
#include <pybind11/stl_bind.h>
#include "Tasks.h"
#include "Simulator.h"



//by using this, pybind11 will avoid copying the data of the vector, and instead, it will pass the pointer
PYBIND11_MAKE_OPAQUE(std::vector<int>);
PYBIND11_MAKE_OPAQUE(std::vector<vector<std::pair<int,int>>>);
PYBIND11_MAKE_OPAQUE(std::vector<State>);
PYBIND11_MAKE_OPAQUE(std::vector<Task>);
PYBIND11_MAKE_OPAQUE(std::unordered_map<int, Task>);


namespace py = pybind11;

PYBIND11_MODULE(MAPF, m ){
    m.doc() = "MAPF wrapper";


    pybind11::class_<Grid>(m, "Grid")
        .def(pybind11::init<std::string>())
        .def_readonly("cols",&Grid::cols)
        .def_readonly("map",&Grid::map)
        .def_readonly("map_name",&Grid::map_name)
        .def_readonly("rows",&Grid::rows);


    pybind11::class_<State>(m, "State")
        .def(pybind11::init<>())
        .def_readonly("location",&State::location)
        .def_readonly("timestep",&State::timestep)
        .def_readonly("orientation",&State::orientation)
        .def(py::self==py::self)
        .def(py::self!=py::self)
        .def(pybind11::init<int,int,int>());


    pybind11::enum_<Action>(m,"Action")
        .value("FW",Action::FW)
        .value("CR",Action::CR)
        .value("CCR",Action::CCR)
        .value("W",Action::W);

    pybind11::class_<ActionModelWithRotate>(m,"ActionModelWithRotate")
        .def(pybind11::init<Grid &>())
        // .def("is_valid",&ActionModelWithRotate::is_valid)
        .def("set_logger",&ActionModelWithRotate::set_logger)
        .def("result_states",&ActionModelWithRotate::result_states);

    pybind11::class_<SharedEnvironment>(m,"SharedEnvironment")
        .def(pybind11::init<>())
        .def_readonly("rows",&SharedEnvironment::rows)
        .def_readonly("cols",&SharedEnvironment::cols)
        .def_readonly("num_of_agents",&SharedEnvironment::num_of_agents)
        .def_readonly("goal_locations",&SharedEnvironment::goal_locations)
        .def_readonly("curr_timestep",&SharedEnvironment::curr_timestep)
        .def_readonly("map",&SharedEnvironment::map)
        .def_readonly("map_name",&SharedEnvironment::map_name)
        .def_readonly("task_pool",&SharedEnvironment::task_pool)
        .def_readonly("new_tasks",&SharedEnvironment::new_tasks)
        .def_readonly("new_freeagents",&SharedEnvironment::new_freeagents)
        .def_readonly("curr_task_schedule",&SharedEnvironment::curr_task_schedule)
        .def_readonly("file_storage_path", &SharedEnvironment::file_storage_path)
        .def_readonly("curr_states",&SharedEnvironment::curr_states)
        .def_readonly("plan_start_time",&SharedEnvironment::plan_start_time)
        .def("plan_current_time",
            [](SharedEnvironment &env) {
                return std::chrono::steady_clock::now();
            }
        );




    //in case that users want to use numpy arrays, or reimplement a gym-like environment, they can modify the pyEnvironment
    // pybind11::class_<pyEnvironment>(m,"pyEnvironment")
    //     .def(pybind11::init<SharedEnvironment *>())
    //     .def("get_rows",&pyEnvironment::get_rows)
    //     .def("get_cols",&pyEnvironment::get_cols)
    //     .def("get_currtimestep",&pyEnvironment::get_currtimestep)
    //     .def("get_map",&pyEnvironment::get_map)
    //     .def("get_map_name", &pyEnvironment::get_map_name)
    //     .def("get_goal_locations",&pyEnvironment::get_goal_locations)
    //     .def("get_num_of_agents",&pyEnvironment::get_num_of_agents)
    //     .def("get_file_storage_path", &pyEnvironment::get_file_storage_path)
    //     .def_readonly("env",&pyEnvironment::env,pybind11::return_value_policy::reference)
    //     .def("get_curr_states",&pyEnvironment::get_curr_states);

    // pybind11::class_<Simulator>(m,"Simulator")
    //     .def(pybind11::init<Grid &,std::vector<int>&,ActionModelWithRotate*>())
    //     .def("move",&Simulator::move)
    //     .def("get_current_state",&Simulator::get_current_state)
    //     .def("get_curr_timestep",&Simulator::get_curr_timestep)
    //     .def("get_all_valid",&Simulator::get_all_valid)
    //     .def("sync_shared_env",&Simulator::sync_shared_env)
    //     .def("actual_path_to_json",&Simulator::actual_path_to_json)
    //     .def("planned_path_to_json",&Simulator::planned_path_to_json)
    //     .def("starts_to_json",&Simulator::starts_to_json)
    //     .def("action_errors_to_json",&Simulator::action_errors_to_json);

    
    pybind11::class_<Task>(m,"Task")
        .def(pybind11::init<int,list<int>,int>())
        .def("get_next_loc",&Task::get_next_loc)
        .def("is_finished",&Task::is_finished)
     
        .def_readonly("task_id",&Task::task_id)
        .def_readonly("locations",&Task::locations)
        .def_readonly("idx_next_loc",&Task::idx_next_loc)
        .def_readonly("t_revealed",&Task::t_revealed)
        .def_readonly("t_completed",&Task::t_completed)
        //agent assigned  must be readwrite so that python side can modify this attribute!
        .def_readwrite("agent_assigned",&Task::agent_assigned);

    pybind11::bind_vector<std::vector<int>>(m, "VectorInt");
    pybind11::bind_vector<std::vector<vector<std::pair<int,int>>>>(m, "VectorGoals");
    pybind11::bind_vector<std::vector<State>>(m, "VectorState");
    pybind11::bind_vector<std::vector<Task>>(m, "VectorTask");
    pybind11::bind_map<std::unordered_map<int, Task>>(m, "MapTask");
}
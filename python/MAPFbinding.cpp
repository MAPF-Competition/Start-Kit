#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include <pybind11/numpy.h>

#include "Grid.h"
#include "SharedEnv.h"
#include "MAPFPlanner.h"
#include "States.h"
#include "pyEnvironment.hpp"


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

    pybind11::class_<SharedEnvironment>(m,"SharedEnvironment")
        .def(pybind11::init<>())
        .def_readonly("rows",&SharedEnvironment::rows)
        .def_readonly("cols",&SharedEnvironment::cols)
        .def_readonly("num_of_agents",&SharedEnvironment::num_of_agents)
        .def_readonly("goal_locations",&SharedEnvironment::goal_locations)
        .def_readonly("curr_timestep",&SharedEnvironment::curr_timestep)
        .def_readonly("map",&SharedEnvironment::map)
        .def_readonly("curr_states",&SharedEnvironment::curr_states);




    //in case that users want to use numpy arrays
    pybind11::class_<pyEnvironment>(m,"pyEnvironment")
        .def(pybind11::init<SharedEnvironment *>())
        .def("get_rows",&pyEnvironment::get_rows)
        .def("get_cols",&pyEnvironment::get_cols)
        .def("get_currtimestep",&pyEnvironment::get_currtimestep)
        .def("get_map",&pyEnvironment::get_map)
        .def("get_goal_locations",&pyEnvironment::get_goal_locations)
        .def("get_num_of_agents",&pyEnvironment::get_num_of_agents)
        .def_readonly("env",&pyEnvironment::env,pybind11::return_value_policy::reference)
        .def("get_curr_states",&pyEnvironment::get_curr_states);

    

    // pybind11::class_<MAPFPlanner>(m,"MAPFPlanner")
    //     .def(pybind11::init<>())
    //     .def(pybind11::init<SharedEnvironment*>())
    //     .def("initialize",&MAPFPlanner::initialize)
    //     .def("plan",&MAPFPlanner::plan);







}
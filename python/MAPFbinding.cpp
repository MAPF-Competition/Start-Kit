#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include <pybind11/numpy.h>

#include "Grid.h"
#include "SharedEnv.h"
#include "MAPFPlanner.h"
#include "States.h"


namespace py = pybind11;

PYBIND11_MODULE(MAPF, m ){
    m.doc() = "MAPF wrapper";


    pybind11::class_<Grid>(m, "Grid")
        .def(pybind11::init<std::string>())
        .def_readonly("cols",&Grid::cols)
        .def_readonly("map",&Grid::map)
        .def_readonly("map_name",&Grid::map_name)
        .def_readonly("rows",&Grid::rows);

    // pybind11::class_<MAPFPlanner>(m,"Grid")
    //     .def(pybind11::init<>())
    //     .def("initialize",&MAPFPlanner::initialize);
    //     .def("")

    pybind11::class_<State>(m, "State")
        .def(pybind11::init<>())
        .def_readonly("location",&State::location)
        .def_readonly("timestep",&State::timestep)
        .def_readonly("orientation",&State::orientation)
        .def(py::self==py::self)
        .def(py::self!=py::self)
        .def(pybind11::init<int,int,int>());

    pybind11::class_<SharedEnvironment>(m,"SharedEnvironment")
        .def(pybind11::init<>())
        .def_readonly("rows",&SharedEnvironment::rows)
        .def_readonly("cols",&SharedEnvironment::cols)
        .def_readonly("num_of_agents",&SharedEnvironment::num_of_agents)
        .def_readonly("goal_locations",&SharedEnvironment::goal_locations)
        .def_readonly("curr_timestep",&SharedEnvironment::curr_timestep)
        .def_readonly("curr_states",&SharedEnvironment::curr_states);




}
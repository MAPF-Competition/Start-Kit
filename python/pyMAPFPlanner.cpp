#include"pyMAPFPlanner.hpp"


pyMAPFPlanner::pyMAPFPlanner():MAPFPlanner(){
    auto sys=pybind11::module_::import("sys");
    py_env=new pyEnvironment(env);
    sys.attr("path").attr("append")("./python");
    auto py_mapf_planner_module=pybind11::module_::import("pyMAPFPlanner");
    std::cout<<"trying to import pyMAPFPlanner module"<<std::endl;
    // py_planner=py_mapf_planner_module.attr("pyMAPFPlanner")(py_env);
    py_planner=py_mapf_planner_module.attr("pyMAPFPlanner")(env);
    std::cout<<"pyMAPF Planner Created!"<<std::endl;
}


void pyMAPFPlanner::initialize(int preprocess_time_limit){
    py_planner.attr("initialize")(preprocess_time_limit);
    // env=py_planner.attr("env").cast<SharedEnvironment*>();
}


std::vector<Action> pyMAPFPlanner::plan(int time_limit){
    return py_planner.attr("plan")(time_limit).cast<std::vector<Action>>();
}



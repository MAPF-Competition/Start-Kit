#include"pyMAPFPlanner.hpp"


pyMAPFPlanner::pyMAPFPlanner():MAPFPlanner(){
    auto sys=pybind11::module_::import("sys");
    py_env=new pyEnvironment(env);
    sys.attr("path").attr("append")("./python");
    auto py_mapf_planner_module=pybind11::module_::import("pyMAPFPlanner");
    std::cout<<"trying to import pyMAPFPlanner module"<<std::endl;
    // py_planner=py_mapf_planner_module.attr("pyMAPFPlanner")(py_env);
    py_planner=py_mapf_planner_module.attr("pyMAPFPlanner")(py_env);
    std::cout<<"pyMAPF Planner Created!"<<std::endl;
}


void pyMAPFPlanner::initialize(int preprocess_time_limit){
    py_planner.attr("initialize")(preprocess_time_limit);
    // env=py_planner.attr("env").cast<SharedEnvironment*>();
}


std::vector<Action> pyMAPFPlanner::plan(int time_limit){
    auto action_object=py_planner.attr("plan")(time_limit);
    try{
        return action_object.cast<std::vector<Action>>();
    }   
    catch(pybind11::cast_error e){
        std::vector<int> tmp_action=action_object.cast<std::vector<int>>();
        std::vector<Action> actions;
        for(auto &ai:tmp_action){
            actions.push_back(static_cast<Action>(ai));
        }
        return actions;
    }
}



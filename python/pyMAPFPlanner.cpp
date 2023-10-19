#include"pyMAPFPlanner.hpp"
#include <unistd.h>

pyMAPFPlanner::pyMAPFPlanner():MAPFPlanner(){
    // pybind11::module::import("torch").attr("cuda").attr("init")();
    auto sys=pybind11::module_::import("sys");
    py_env=new pyEnvironment(env);
    std::ifstream configFile("config.json");
    
    //default
    std::cout<<"setting to default python path: ./python, ../python, ./build "<<std::endl;
    sys.attr("path").attr("append")("./python");
    sys.attr("path").attr("append")("../python");
    sys.attr("path").attr("append")("./build");
    
    if(configFile){
        nlohmann::json configData;
        try{
            configFile>>configData;
            if(configData.contains("python_path")){
                std::string python_path=configData["python_path"];
                std::cout<<"addinng "<<python_path<<" to system path"<<std::endl;
                sys.attr("path").attr("append")(python_path);
            }
        }
        catch(const nlohmann::json::parse_error& e){
            std::cerr << "Error: Failed to parse config file. " << e.what() << std::endl;
        }
    }
    
    std::cout<<"trying to import pyMAPFPlanner module"<<std::endl;
    auto py_mapf_planner_module=pybind11::module_::import("pyMAPFPlanner");

    std::cout<<"trying to create pyMAPFPlanner"<<std::endl;
    py_planner=py_mapf_planner_module.attr("pyMAPFPlanner")(py_env);

    std::cout<<"pyMAPF Planner Created!"<<std::endl;
}


void pyMAPFPlanner::initialize(int preprocess_time_limit){
    pybind11::gil_scoped_release release;
    pybind11::gil_scoped_acquire acquire;
    std::cout<<"pyMAPFPlanner begin to initialize"<<std::endl;
    py_planner.attr("initialize")(preprocess_time_limit);

    // env=py_planner.attr("env").cast<SharedEnvironment*>();
}


void pyMAPFPlanner::plan(int time_limit,std::vector<Action> &plan){
    pybind11::gil_scoped_release release;
    pybind11::gil_scoped_acquire acquire;
    std::cout<<"calling python planner"<<std::endl;

    auto action_object=py_planner.attr("plan")(time_limit);

    try{
        plan=action_object.cast<std::vector<Action>>();
        // assert(plan.empty()==false);
        // return actions;
    }   
    catch(pybind11::cast_error e){
        plan.clear();
        std::vector<int> tmp_action=action_object.cast<std::vector<int>>();
        // std::vector<Action> actions;
        for(auto &ai:tmp_action){
            plan.push_back(static_cast<Action>(ai));
        }
        // assert(actions.empty()==false);
        // return actions;
    }
    // pybind11::gil_scoped_release release;
 
}



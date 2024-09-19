#include "pyTaskScheduler.hpp"


pyTaskScheduler::pyTaskScheduler(SharedEnvironment* env): TaskScheduler(env)
{
    auto sys=pybind11::module_::import("sys");
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

    std::cout<<"trying to import pyTaskScheduler module"<<std::endl;
    auto py_task_scheduler_module=pybind11::module_::import("pyTaskScheduler");

    std::cout<<"trying to create pyTaskScheduler"<<std::endl;
    py_scheduler=py_task_scheduler_module.attr("pyTaskScheduler")(env);

}

void pyTaskScheduler::initialize(int preprocess_time_limit)
{
    pybind11::gil_scoped_release release;
    pybind11::gil_scoped_acquire acquire;
    std::cout<<"pyTaskScheduler begin to initialize"<<std::endl;
    py_scheduler.attr("initialize")(preprocess_time_limit);

}


void pyTaskScheduler::plan(int time_limit, std::vector<int> & proposed_schedule)
{
    pybind11::gil_scoped_release release;
    pybind11::gil_scoped_acquire acquire;
    // std::cout<<"calling python scheduler"<<std::endl;

    auto proposed_schedule_object=py_scheduler.attr("plan")(time_limit);

    try{
        proposed_schedule=proposed_schedule_object.cast<std::vector<int>>();
        // assert(plan.empty()==false);
    }
    catch(const pybind11::cast_error& e){
        std::cerr << "Error: Failed to cast python object to c++ object. " << e.what() << std::endl;
    }
    catch(const pybind11::error_already_set& e){
        std::cerr << "Error: Failed to call python function. " << e.what() << std::endl;
    }
    catch(const std::exception& e){
        std::cerr << "Error: Failed to cast python object to c++ object. " << e.what() << std::endl;
    }
    catch(...){
        std::cerr << "Error: Failed to cast python object to c++ object. " << std::endl;
    }

}


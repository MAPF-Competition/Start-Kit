#include "CompetitionSystem.h"
#include "Evaluation.h"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include "nlohmann/json.hpp"
#include <signal.h>


namespace po = boost::program_options;
using json = nlohmann::json;

po::variables_map vm;
BaseSystem* system_ptr = nullptr;

void sigint_handler(int a)
{
    fprintf(stdout, "stop the simulation...\n");
    if (!vm["evaluationMode"].as<bool>()){
        system_ptr->saveResults(vm["output"].as<std::string>());
    }

    _exit(0);
}

int main(int argc, char** argv) {
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("inputFolder", po::value<std::string>()->default_value("."), "input folder")
        ("inputFile,i", po::value<std::string>()->required(), "input file name")
        ("plannerPath", po::value<std::string>()->default_value("./exp/test_planner.txt"), "planner path file name")
        ("actualPath", po::value<std::string>()->default_value("./exp/test_actual.txt"), "actual path file name")
        ("output,o", po::value<std::string>()->default_value("./exp/test.json"), "output file name")
        ("evaluationMode", po::value<bool>()->default_value(false), "evaluate an existing output file")
        ("simulationTime", po::value<int>()->default_value(5000), "run simulation")
        ("logFile,l", po::value<std::string>(), "issue log file name")
        ;
    clock_t start_time = clock();
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    po::notify(vm);

    std::string base_folder = vm["inputFolder"].as<std::string>();
    if (base_folder.size() > 0 && base_folder.back()!='/'){
        base_folder += "/";
    }

    Logger* logger = new Logger();
    if (vm.count("logFile"))
        logger->set_logfile(vm["logFile"].as<std::string>());


    MAPFPlanner* planner = nullptr;

    if (vm["evaluationMode"].as<bool>()){
        logger->log_info("running the evaluation mode");
        planner = new DummyPlanner(vm["output"].as<std::string>());
    }else{
        planner = new MAPFPlanner();
    }

    auto input_json_file = base_folder + vm["inputFile"].as<std::string>();
    json data;
    std::ifstream f(input_json_file);
    try{
        data = json::parse(f);
    }
    catch(json::parse_error error ) {
        std::cerr << "Failed to load " << input_json_file << std::endl;
        std::cerr << "Message: " << error.what() << std::endl;
        exit(1);
    }

    Grid grid(base_folder + data["mapFile"].get<std::string>());
    ActionModelWithRotate* model = new ActionModelWithRotate(grid);
    model->set_logger(logger);

    std::vector<int> agents = read_int_vec(base_folder + read_param_json<std::string>(data, "agentFile"));
    std::vector<int> tasks = read_int_vec(base_folder + read_param_json<std::string>(data, "taskFile"));
    std::cout << agents.size() << " agents and " << tasks.size() << " tasks"<< std::endl;

    std::string task_assignment_strategy = data["taskAssignmentStrategy"].get<std::string>();
    if (task_assignment_strategy=="greedy"){
        system_ptr = new TaskAssignSystem(grid, planner, agents, tasks, model);
    } else if (task_assignment_strategy=="roundrobin"){
        std::vector<vector<int>> assigned_tasks(agents.size());
        for(int i = 0; i < tasks.size(); i++){
            assigned_tasks[i%agents.size()].push_back(tasks[i]);
        }
        system_ptr = new FixedAssignSystem(grid, planner, agents, assigned_tasks, model);
    } else{
        std::cerr << "unkown task assignment strategy " << data["taskAssignmentStrategy"].get<std::string>() << std::endl;
        logger->log_fatal("unkown task assignment strategy " + data["taskAssignmentStrategy"].get<std::string>());
        exit(1);
    }

    system_ptr->set_logger(logger);
    system_ptr->set_plan_time_limit(read_param_json<int>(data, "planTimeLimit", 5));
    system_ptr->set_preprocess_time_limit(read_param_json<int>(data, "preprocessTimeLimit", 10));
    system_ptr->set_num_tasks_reveal(read_param_json<int>(data, "numTasksReveal", 1));

    signal(SIGINT, sigint_handler);

    system_ptr->simulate(vm["simulationTime"].as<int>());

    if (!vm["evaluationMode"].as<bool>()){
        system_ptr->saveResults(vm["output"].as<std::string>());
    }

    delete model;
    delete planner->env;
    delete system_ptr;
    return 0;
}

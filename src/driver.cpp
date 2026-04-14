#include "CompetitionSystem.h"
#include "Evaluation.h"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include "nlohmann/json.hpp"
#include <signal.h>
#include <climits>
#include <memory>
#include <stdexcept>


#include <pybind11/embed.h>
#include "pyEntry.hpp"

namespace po = boost::program_options;
using json = nlohmann::json;

po::variables_map vm;
std::unique_ptr<BaseSystem> system_ptr;

void sigint_handler(int a)
{
    fprintf(stdout, "stop the simulation...\n");
    system_ptr->saveResults(
        vm["output"].as<std::string>(),
        vm["outputScreen"].as<int>(),
        vm["prettyPrintJson"].as<bool>());
    _exit(0);
}


int main(int argc, char **argv)
{
    pybind11::initialize_interpreter();
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()("help", "produce help message")
        ("actionMoveTimeLimit,a", po::value<int>()->default_value(100), "the  time limit for move one action in milliseconds")
        ("outputScreen,c", po::value<int>()->default_value(1), "the level of details in the output file, 1--showing all the output, 2--only actual plans, task completion and summary, 3--only summary")
        ("logDetailLevel,d", po::value<int>()->default_value(1), "the minimum severity level of log messages to display, 1--showing all the messages, 2--showing warnings and fatal errors, 3--showing fatal errors only")
        ("fileStoragePath,f", po::value<std::string>()->default_value(""), "the large file storage path")
        ("inputFile,i", po::value<std::string>()->required(), "input file name")
        ("logFile,l", po::value<std::string>()->default_value(""), "redirect stdout messages into the specified log file")
        ("initialPlanTimeLimit,n", po::value<int>()->default_value(1000), "the initial time limit for planner in milliseconds")
        ("output,o", po::value<std::string>()->default_value("./output.json"), "output results from the evaluation into a JSON formated file. If no file specified, the default name is 'output.json'")
        ("prettyPrintJson", po::bool_switch()->default_value(false), "pretty-print the output JSON instead of writing it on one line")
        ("preprocessTimeLimit,p", po::value<int>()->default_value(30000), "the time limit for preprocessing in milliseconds")
        ("disableStagedActionValidation", po::bool_switch()->default_value(false), "disable validation that executor staged actions remain a prefix of the previous staged actions plus the new plan")
        ("simulationTime,s", po::value<int>()->default_value(5000), "run simulation")
        ("taskTrendFile", po::value<std::string>()->default_value(""), "write task-finish trend snapshots to a text file")
        ("taskTrendInterval", po::value<int>()->default_value(100), "sample task-finish trend every N ticks")
        ("planCommTimeLimit,t", po::value<int>()->default_value(1000), "the minimal communication time limit for planner in milliseconds")
        ("outputActionWindow,w", po::value<int>()->default_value(1000), "output results from the evaluation into a JSON formated file. If no file specified, the default name is 'output.json'")
        ("executorProcessPlanTimeLimit,x", po::value<int>()->default_value(100), "the time limit for process new plan in milliseconds")
        ("plannerPython", po::value<bool>()->default_value(false), "use Python MAPFPlanner implementation")
        ("schedulerPython", po::value<bool>()->default_value(false), "use Python TaskScheduler implementation")
        ("executorPython", po::value<bool>()->default_value(false), "use Python Executor implementation")
        ;
    clock_t start_time = clock();
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }

    po::notify(vm);

    boost::filesystem::path p(vm["inputFile"].as<std::string>());
    boost::filesystem::path dir = p.parent_path();
    std::string base_folder = dir.string();
    if (base_folder.size() > 0 && base_folder.back() != '/')
    {
        base_folder += "/";
    }

    int log_level = vm["logDetailLevel"].as<int>();
    if (log_level <= 1)
        log_level = 2; //info
    else if (log_level == 2)
        log_level = 3; //warning
    else
        log_level = 5; //fatal

    Logger *logger = new Logger(vm["logFile"].as<std::string>(),log_level);

    std::filesystem::path filepath(vm["output"].as<std::string>());
    if (filepath.parent_path().string().size() > 0 && !std::filesystem::is_directory(filepath.parent_path()))
    {
        logger->log_fatal("output directory does not exist",0);
        _exit(1);
    }


    Entry *planner = nullptr;
    Executor *executor = nullptr;

    bool use_py_planner   = vm["plannerPython"].as<bool>();
    bool use_py_scheduler = vm["schedulerPython"].as<bool>();
    bool use_py_executor  = vm["executorPython"].as<bool>();

    if (use_py_planner || use_py_scheduler || use_py_executor) {
        auto *pe = new pyEntry(use_py_planner, use_py_scheduler, use_py_executor);
        executor = pe->get_executor();
        planner = pe;
    } else {
        planner = new Entry();
        executor = new Executor();
    }

    auto input_json_file = vm["inputFile"].as<std::string>();
    json data;
    std::ifstream f(input_json_file);
    try
    {
        data = json::parse(f);
    }
    catch (json::parse_error error)
    {
        std::cerr << "Failed to load " << input_json_file << std::endl;
        std::cerr << "Message: " << error.what() << std::endl;
        exit(1);
    }

    auto map_path = read_param_json<std::string>(data, "mapFile");
    Grid grid(base_folder + map_path);

    planner->env->map_name = map_path.substr(map_path.find_last_of("/") + 1);
    executor->env->map_name = planner->env->map_name;


    string file_storage_path = vm["fileStoragePath"].as<std::string>();
    if (file_storage_path==""){
      char const* tmp = getenv("LORR_LARGE_FILE_STORAGE_PATH");
      if ( tmp != nullptr ) {
        file_storage_path = string(tmp);
      }
    }

    // check if the path exists;
    if (file_storage_path!="" &&!std::filesystem::exists(file_storage_path))
    {
      std::ostringstream stringStream;
      stringStream << "fileStoragePath (" << file_storage_path << ") is not valid";
      logger->log_warning(stringStream.str());
    }

    planner->env->file_storage_path = file_storage_path;
    executor->env->file_storage_path = file_storage_path;

    float agent_size = read_param_json<float>(data, "agentSize", 1.0f);
    if (agent_size <= 0.0f)
    {
        throw std::invalid_argument("agentSize should be a positive number");
    }

    ActionModelWithRotate *model = new ActionModelWithRotate(grid, agent_size);
    model->set_logger(logger);

    int team_size = read_param_json<int>(data, "teamSize");
    DelayConfig delay_config;
    try
    {
        delay_config = parse_delay_config(data);
    }
    catch (const std::invalid_argument& error)
    {
        logger->log_fatal(error.what(), 0);
        _exit(1);
    }

    const std::string delay_event_distribution =
        delay_config.eventModel == DelayConfig::EventModel::Bernoulli ? "bernoulli" : "poisson";
    const std::string delay_time_distribution =
        delay_config.durationModel == DelayConfig::DurationModel::Uniform ? "uniform" : "gaussian";

    planner->env->delay_event_distribution = delay_event_distribution;
    planner->env->delay_time_distribution = delay_time_distribution;
    executor->env->delay_event_distribution = delay_event_distribution;
    executor->env->delay_time_distribution = delay_time_distribution;

    std::vector<int> agents = read_int_vec(base_folder + read_param_json<std::string>(data, "agentFile"), team_size);
    std::vector<list<int>> tasks = read_int_vec(base_folder + read_param_json<std::string>(data, "taskFile"));
    if (agents.size() > tasks.size())
        logger->log_warning("Not enough tasks for robots (number of tasks < team size)");

    system_ptr = std::make_unique<BaseSystem>(grid, planner, executor, agents, tasks, model, read_param_json<int>(data, "agentCounter", 10));

    system_ptr->set_logger(logger);
    const int task_trend_interval = vm["taskTrendInterval"].as<int>();
    if (task_trend_interval <= 0)
    {
        logger->log_fatal("taskTrendInterval must be a positive integer", 0);
        _exit(1);
    }
    system_ptr->set_task_trend_output(vm["taskTrendFile"].as<std::string>(), task_trend_interval);

    system_ptr->set_logger(logger);
    system_ptr->set_plan_time_limit(vm["initialPlanTimeLimit"].as<int>(),vm["planCommTimeLimit"].as<int>(),vm["actionMoveTimeLimit"].as<int>(),vm["executorProcessPlanTimeLimit"].as<int>());
    system_ptr->set_preprocess_time_limit(vm["preprocessTimeLimit"].as<int>());
    system_ptr->set_staged_action_validation_enabled(!vm["disableStagedActionValidation"].as<bool>());

    system_ptr->set_num_tasks_reveal(read_param_json<float>(data, "numTasksReveal", 1));

    try
    {
        system_ptr->set_delay_generator(std::make_unique<DelayGenerator>(delay_config, team_size));
    }
    catch (const std::invalid_argument& error)
    {
        logger->log_fatal(error.what(), 0);
        _exit(1);
    }

    signal(SIGINT, sigint_handler);

    // Release GIL so worker threads in simulate() can acquire it for Python calls
    {
        pybind11::gil_scoped_release release;
        system_ptr->simulate(vm["simulationTime"].as<int>(),100);
    }

    system_ptr->saveResults(
        vm["output"].as<std::string>(),
        vm["outputScreen"].as<int>(),
        vm["prettyPrintJson"].as<bool>());

    delete model;
    delete logger;
    _exit(0);
}

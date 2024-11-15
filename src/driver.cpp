#include "CompetitionSystem.h"
#include "Evaluation.h"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include "nlohmann/json.hpp"
#include <signal.h>
#include <climits>
#include <memory>


#ifdef PYTHON
#if PYTHON
#include "pyMAPFPlanner.hpp"
#include <pybind11/embed.h>
#include "pyEntry.hpp"
#include "pyTaskScheduler.hpp"
#endif
#endif

namespace po = boost::program_options;
using json = nlohmann::json;

po::variables_map vm;
std::unique_ptr<BaseSystem> system_ptr;


void sigint_handler(int a)
{
    fprintf(stdout, "stop the simulation...\n");
    system_ptr->saveResults(vm["output"].as<std::string>(),vm["outputScreen"].as<int>());
    _exit(0);
}


int main(int argc, char **argv)
{
#ifdef PYTHON
#if PYTHON
    pybind11::initialize_interpreter();
#endif
#endif
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()("help", "produce help message")
        ("inputFile,i", po::value<std::string>()->required(), "input file name")
        ("output,o", po::value<std::string>()->default_value("./output.json"), "output results from the evaluation into a JSON formated file. If no file specified, the default name is 'output.json'")
        ("outputScreen,c", po::value<int>()->default_value(1), "the level of details in the output file, 1--showing all the output, 2--ignore the events and tasks, 3--ignore the events, tasks, errors, planner times, starts and paths")
        ("evaluationMode,m", po::value<bool>()->default_value(false), "evaluate an existing output file")
        ("simulationTime,s", po::value<int>()->default_value(5000), "run simulation")
        ("fileStoragePath,f", po::value<std::string>()->default_value(""), "the large file storage path")
        ("planTimeLimit,t", po::value<int>()->default_value(1000), "the time limit for planner in milliseconds")
        ("preprocessTimeLimit,p", po::value<int>()->default_value(30000), "the time limit for preprocessing in milliseconds")
        ("logFile,l", po::value<std::string>()->default_value(""), "redirect stdout messages into the specified log file")
        ("logDetailLevel,d", po::value<int>()->default_value(1), "the minimum severity level of log messages to display, 1--showing all the messages, 2--showing warnings and fatal errors, 3--showing fatal errors only");
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

#ifdef PYTHON
#if PYTHON
        planner = new PyEntry();
#else
        planner = new Entry();
#endif
#endif

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


    string file_storage_path = vm["fileStoragePath"].as<std::string>();
    if (file_storage_path==""){
      char const* tmp = getenv("LORR_LARGE_FILE_STORAGE_PATH");
      if ( tmp != nullptr ) {
        file_storage_path = string(tmp);
      }
    }

    // check if the path exists;
    if (file_storage_path!="" &&!std::filesystem::exists(file_storage_path)){
      std::ostringstream stringStream;
      stringStream << "fileStoragePath (" << file_storage_path << ") is not valid";
      logger->log_warning(stringStream.str());
    }
    planner->env->file_storage_path = file_storage_path;

    ActionModelWithRotate *model = new ActionModelWithRotate(grid);
    model->set_logger(logger);

    int team_size = read_param_json<int>(data, "teamSize");

    std::vector<int> agents = read_int_vec(base_folder + read_param_json<std::string>(data, "agentFile"), team_size);
    std::vector<list<int>> tasks = read_int_vec(base_folder + read_param_json<std::string>(data, "taskFile"));
    if (agents.size() > tasks.size())
        logger->log_warning("Not enough tasks for robots (number of tasks < team size)");

    system_ptr = std::make_unique<BaseSystem>(grid, planner, agents, tasks, model);

    system_ptr->set_logger(logger);
    system_ptr->set_plan_time_limit(vm["planTimeLimit"].as<int>());
    system_ptr->set_preprocess_time_limit(vm["preprocessTimeLimit"].as<int>());

    system_ptr->set_num_tasks_reveal(read_param_json<float>(data, "numTasksReveal", 1));

    signal(SIGINT, sigint_handler);

    system_ptr->simulate(vm["simulationTime"].as<int>());


    system_ptr->saveResults(vm["output"].as<std::string>(),vm["outputScreen"].as<int>());

    delete model;
    delete logger;
    _exit(0);
}

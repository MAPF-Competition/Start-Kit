#include "CompetitionSystem.h"
#include "Evaluation.h"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include "nlohmann/json.hpp"
#include <signal.h>


namespace po = boost::program_options;
po::variables_map vm;
BaseSystem* system_ptr = nullptr;

void sigint_handler(int a)
{
  fprintf(stdout, "stop the simulation...\n");

  if (!vm["evaluation"].as<bool>()){
    system_ptr->saveResults(vm["output"].as<std::string>());
  }

  _exit(0);
}

using json = nlohmann::json;

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
		("cutoffTime,t", po::value<int>()->default_value(60), "cutoff time (seconds)")
		("seed,d", po::value<int>(), "random seed")
		("screen,s", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")
		("simulation_time", po::value<int>()->default_value(5000), "run simulation")
		("evaluation", po::value<bool>()->default_value(false), "evaluate an existing output file")
    ("issueLog", po::value<std::string>(), "issue log file name")
	;
	clock_t start_time = clock();
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 1;
	}

	po::notify(vm);

    // make dictionary
	//boost::filesystem::path dir(vm["output"].as<std::string>() +"/");
	//boost::filesystem::create_directories(dir);

	MAPFPlanner* planner = nullptr;

    if (vm["evaluation"].as<bool>()){
      planner = new DummyPlanner(vm["output"].as<std::string>());
    }else{
      planner = new MAPFPlanner();
    }

	std::ifstream f(vm["inputFolder"].as<std::string>() + "/" + vm["inputFile"].as<std::string>());
	json data = json::parse(f);

	Grid grid(vm["inputFolder"].as<std::string>() + "/" + data["map_file"].get<std::string>());

	std::vector<int> agents = read_int_vec(vm["inputFolder"].as<std::string>() + "/" + data["agent_file"].get<std::string>());
	std::vector<int> tasks = read_int_vec(vm["inputFolder"].as<std::string>() + "/" + data["task_file"].get<std::string>());

	std::cout << agents.size() << " agents and " << tasks.size() << " tasks"<< std::endl;

	ActionModelWithRotate* model = new ActionModelWithRotate(grid);


  if (data["task_assignment_strategy"].get<std::string>()=="greedy"){
    system_ptr = new TaskAssignSystem(grid, planner, agents, tasks, model);
  } else if (data["task_assignment_strategy"].get<std::string>()=="roundrobin"){
    std::vector<vector<int>> assigned_tasks(agents.size());
    for(int i = 0; i < tasks.size(); i++){
      assigned_tasks[i%agents.size()].push_back(tasks[i]);
    }

    system_ptr = new FixedAssignSystem(grid, planner, agents, assigned_tasks, model);
  } else{
    std::cerr << "unkown task_assignment_strategy " << data["task_assignment_strategy"].get<std::string>() << std::endl;
    exit(1);
  }

  if (data.contains("plan_time_limit") && data["plan_time_limit"].is_number_integer()){
    system_ptr->set_plan_time_limit(data["plan_time_limit"].get<int>());
  }

  if (data.contains("preprocess_time_limit") && data["preprocess_time_limit"].is_number_integer()){
    system_ptr->set_preprocess_time_limit(data["preprocess_time_limit"].get<int>());
  }

  system_ptr->set_num_tasks_reveal(data["num_tasks_reveal"].get<int>());
  if (vm.count("issueLog"))
    system_ptr->setLoggerFile(vm["issueLog"].as<std::string>());

  signal(SIGINT, sigint_handler);

  system_ptr->simulate(vm["simulation_time"].as<int>());

  //system_ptr->savePaths(vm["plannerPath"].as<std::string>(),1);
  //system_ptr->savePaths(vm["actualPath"].as<std::string>(),0);
  //system_ptr->saveErrors("./exp/error.txt");
  if (!vm["evaluation"].as<bool>()){
    system_ptr->saveResults(vm["output"].as<std::string>());
  }
  //system_ptr->saveSimulationIssues(vm["issueLog"].as<std::string>());

  delete model;
	delete planner->env;
  delete system_ptr;
	return 0;
}

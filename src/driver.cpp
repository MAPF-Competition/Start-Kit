#include "CompetitionSystem.h"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include "nlohmann/json.hpp"

std::vector<int> read_int_vec(string fname){
  std::vector<int> res;
	string line;
	std::ifstream myfile(fname.c_str());
	if (!myfile.is_open()) return {};

	getline(myfile, line);
  while (!myfile.eof() && line[0] == '#') {
    getline(myfile, line);
  }

  boost::char_separator<char> sep(",");
  boost::tokenizer<boost::char_separator<char>> tok(line, sep);
  boost::tokenizer<boost::char_separator<char>>::iterator beg = tok.begin();

  int num_of_int = atoi((*beg).c_str());
  // My benchmark
  for (int i = 0; i < num_of_int; i++) {

    getline(myfile, line);
    while (!myfile.eof() && line[0] == '#'){
      getline(myfile, line);
    }
    boost::tokenizer<boost::char_separator<char>> tok(line, sep);
    boost::tokenizer<boost::char_separator<char>>::iterator beg = tok.begin();
    // read start [row,col] for agent i
    res.push_back(atoi((*beg).c_str()));

  }
  myfile.close();

	return res;
}



using json = nlohmann::json;

int main(int argc, char** argv) {
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("input,i", po::value<std::string>()->required(), "input file")
		("plannerPath", po::value<std::string>()->default_value("./exp/test_planner.txt"), "planner path file name")
		("actualPath", po::value<std::string>()->default_value("./exp/test_actual.txt"), "actual path file name")
		("output,o", po::value<std::string>()->default_value("./exp/test.json"), "output file name")
		("cutoffTime,t", po::value<int>()->default_value(60), "cutoff time (seconds)")
		("seed,d", po::value<int>(), "random seed")
		("screen,s", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")
		("simulation_time", po::value<int>()->default_value(5000), "run simulation")
		("checkconf", po::value<bool>()->default_value(true), "consider conflict")
	;
	clock_t start_time = clock();
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 1;
	}

	po::notify(vm);

    // make dictionary
	//boost::filesystem::path dir(vm["output"].as<std::string>() +"/");
	//boost::filesystem::create_directories(dir);

	MAPFPlanner* planner = new MAPFPlanner();

	std::ifstream f(vm["input"].as<std::string>());
	json data = json::parse(f);

	Grid grid(data["map_file"].get<std::string>());

	std::vector<int> agents = read_int_vec(data["agent_file"].get<std::string>());
	std::vector<int> tasks = read_int_vec(data["task_file"].get<std::string>());

	std::cout << agents.size() << " agents and " << tasks.size() << " tasks"<< std::endl;

	ActionModelWithRotate* model = new ActionModelWithRotate(grid);

  BaseSystem* system_ptr = nullptr;

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

  system_ptr->set_num_tasks_reveal(data["num_tasks_reveal"].get<int>());
  system_ptr->simulate(vm["simulation_time"].as<int>());

  //system_ptr->savePaths(vm["plannerPath"].as<std::string>(),1);
  //system_ptr->savePaths(vm["actualPath"].as<std::string>(),0);
  //system_ptr->saveErrors("./exp/error.txt");
  system_ptr->saveResults(vm["output"].as<std::string>());

  delete model;
	delete planner->env;
  delete system_ptr;
	return 0;
}

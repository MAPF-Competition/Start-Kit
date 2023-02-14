#include "CompetitionSystem.h"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>


int main(int argc, char** argv) {
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("map,m", po::value<std::string>()->required(), "input map file")
		("task", po::value<std::string>()->required(), "input task file")
		("plannerPath,po", po::value<std::string>()->default_value("./exp/test_planner"), "planner path file name")
		("actualPath,ao", po::value<std::string>()->default_value("./exp/test_actual"), "actual path file name")
		("output,o", po::value<std::string>()->default_value("./exp/test"), "output folder name")
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
	boost::filesystem::path dir(vm["output"].as<std::string>() +"/");
	boost::filesystem::create_directories(dir);

  MAPFPlanner* planner = new MAPFPlanner();

  Grid grid(vm["map"].as<std::string>());

  Validator* validator = nullptr;
  if (vm["checkconf"].as<bool>()){
    validator = new ValidatorRotate(grid);
  }

	CompetitionSystem system(grid, vm["task"].as<std::string>(), planner, validator);
	system.simulate(vm["simulation_time"].as<int>());

	system.savePaths(vm["plannerPath"].as<std::string>(),1);
	system.savePaths(vm["actualPath"].as<std::string>(),0);

  if (validator != nullptr){delete validator;}
	delete planner->env;
	return 0;
}

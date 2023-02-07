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

	CompetitionSystem system(planner);
	system.check_collisions = vm["checkconf"].as<bool>();
	system.load_map(vm["map"].as<std::string>());
	system.load_agent_tasks(vm["task"].as<std::string>());
	system.simulate(vm["simulation_time"].as<int>());

	system.savePaths(vm["plannerPath"].as<std::string>(),1);
	system.savePaths(vm["actualPath"].as<std::string>(),0);

	delete planner->env;
	return 0;

	// if (vm["scenario"].as<string>() == "KIVA")
	// {
	// 	KivaGrid G;
	// 	if (!G.load_map(vm["map"].as<std::string>()))
	// 		return -1;
	// 	MAPFSolver* solver = set_solver(G, vm);
	// 	KivaSystem system(G, *solver);
	// 	set_parameters(system, vm);
	// 	G.preprocessing(system.consider_rotation);
	// 	system.simulate(vm["simulation_time"].as<int>());
	// 	return 0;
	// }
	// else if (vm["scenario"].as<string>() == "ONLINE")
	// {
	// 	OnlineGrid G;
	// 	if (!G.load_map(vm["map"].as<std::string>()))
	// 		return -1;
	// 	MAPFSolver* solver = set_solver(G, vm);
	// 	OnlineSystem system(G, *solver);
	// 	assert(!system.hold_endpoints);
	// 	assert(!system.useDummyPaths);
	// 	set_parameters(system, vm);
	// 	G.preprocessing(system.consider_rotation);
	// 	system.simulate(vm["simulation_time"].as<int>());
	// 	return 0;
	// } 
	// else
	// {
	// 	cout << "Scenario " << vm["scenario"].as<string>() << "does not exist!" << endl;
	// 	return -1;
	// }
}

#include "KivaSystem.h"
#include "OnlineSystem.h"
#include "ID.h"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

std::string solver_name;

void set_parameters(BasicSystem& system, const boost::program_options::variables_map& vm)
{
	system.outfile = vm["output"].as<std::string>();
	system.screen = vm["screen"].as<int>();
	system.log = vm["log"].as<bool>();
	system.num_of_drives = vm["agentNum"].as<int>();
	system.time_limit = vm["cutoffTime"].as<int>();
	system.simulation_window = vm["simulation_window"].as<int>();
	system.planning_window = vm["planning_window"].as<int>();
	system.travel_time_window = vm["travel_time_window"].as<int>();
	system.consider_rotation = vm["rotation"].as<bool>();
	system.k_robust = vm["robust"].as<int>();
	system.hold_endpoints = vm["hold_endpoints"].as<bool>();
	system.useDummyPaths = vm["dummy_paths"].as<bool>();
	if (vm.count("seed"))
		system.seed = vm["seed"].as<int>();
	else
		system.seed = (int)time(0);
	srand(system.seed);
}


MAPFSolver* set_solver(const BasicGraph& G, const boost::program_options::variables_map& vm)
{
	SingleAgentSolver* path_planner;
	MAPFSolver* mapf_solver;
  path_planner = new StateTimeAStar();
	solver_name = vm["solver"].as<string>();
	if (solver_name == "WHCA")
	{
		mapf_solver = new WHCAStar(G, *path_planner);
	}
	else if (solver_name == "LRA")
	{
		mapf_solver = new LRAStar(G, *path_planner);
	}
	else
	{
		cout << "Solver " << solver_name << "does not exist!" << endl;
		exit(-1);
	}

	if (vm["id"].as<bool>())
	{
		return new ID(G, *path_planner, *mapf_solver);
	}
	else
	{
		return mapf_solver;
	}
}


int main(int argc, char** argv) 
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("scenario", po::value<std::string>()->required(), "scenario (KIVA, ONLINE)")
		("map,m", po::value<std::string>()->required(), "input map file")
		("task", po::value<std::string>()->default_value(""), "input task file")
		("output,o", po::value<std::string>()->default_value("../exp/test"), "output folder name")
		("agentNum,k", po::value<int>()->required(), "number of drives")
		("cutoffTime,t", po::value<int>()->default_value(60), "cutoff time (seconds)")
		("seed,d", po::value<int>(), "random seed")
		("screen,s", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")
		("solver", po::value<string>()->default_value("PBS"), "solver (LRA, PBS, WHCA, ECBS)")
		("id", po::value<bool>()->default_value(false), "independence detection")
		("lazyP", po::value<bool>()->default_value(false), "use lazy priority")
		("simulation_time", po::value<int>()->default_value(5000), "run simulation")
		("simulation_window", po::value<int>()->default_value(5), "call the planner every simulation_window timesteps")
		("travel_time_window", po::value<int>()->default_value(0), "consider the traffic jams within the given window")
		("planning_window", po::value<int>()->default_value(INT_MAX / 2),
		        "the planner outputs plans with first planning_window timesteps collision-free")
		("potential_function", po::value<string>()->default_value("NONE"), "potential function (NONE, SOC, IC)")
		("potential_threshold", po::value<double>()->default_value(0), "potential threshold")
		("rotation", po::value<bool>()->default_value(false), "consider rotation")
		("robust", po::value<int>()->default_value(0), "k-robust (for now, only work for PBS)")
		("CAT", po::value<bool>()->default_value(false), "use conflict-avoidance table")
		// ("PG", po::value<bool>()->default_value(false),
		//        "reuse the priority graph of the goal node of the previous search")
		("hold_endpoints", po::value<bool>()->default_value(false),
		        "Hold endpoints from Ma et al, AAMAS 2017")
		("dummy_paths", po::value<bool>()->default_value(false),
				"Find dummy paths from Liu et al, AAMAS 2019")
		("prioritize_start", po::value<bool>()->default_value(true), "Prioritize waiting at start locations")
		("suboptimal_bound", po::value<double>()->default_value(1), "Suboptimal bound for ECBS")
		("log", po::value<bool>()->default_value(false), "save the search trees (and the priority trees)")
		;
	clock_t start_time = clock();
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 1;
	}

	po::notify(vm);

    // check params
    if (vm["hold_endpoints"].as<bool>() or vm["dummy_paths"].as<bool>())
    {
        if (vm["prioritize_start"].as<bool>())
        {
            std::cerr << "Please add --prioritize_start=false to the command since prioritize_start does not work with "
                         "hold endpoints or dummy paths." << endl;
            exit(-1);
        }
        if (vm["hold_endpoints"].as<bool>() and vm["dummy_paths"].as<bool>())
        {
            std::cerr << "Hold endpoints and dummy paths cannot be used simultaneously" << endl;
            exit(-1);
        }
        if (vm["simulation_window"].as<int>() != 1)
        {
            std::cerr << "Hold endpoints and dummy paths can only work when the simulation window is 1" << endl;
            exit(-1);
        }
        if (vm["planning_window"].as<int>() < INT_MAX / 2)
        {
            std::cerr << "Hold endpoints and dummy paths cannot work with planning windows" << endl;
            exit(-1);
        }
    }

    // make dictionary
	boost::filesystem::path dir(vm["output"].as<std::string>() +"/");
	boost::filesystem::create_directories(dir);
	if (vm["log"].as<bool>())
	{
		boost::filesystem::path dir1(vm["output"].as<std::string>() + "/goal_nodes/");
		boost::filesystem::path dir2(vm["output"].as<std::string>() + "/search_trees/");
		boost::filesystem::create_directories(dir1);
		boost::filesystem::create_directories(dir2);
	}


	if (vm["scenario"].as<string>() == "KIVA")
	{
		KivaGrid G;
		if (!G.load_map(vm["map"].as<std::string>()))
			return -1;
		MAPFSolver* solver = set_solver(G, vm);
		KivaSystem system(G, *solver);
		set_parameters(system, vm);
		G.preprocessing(system.consider_rotation);
		system.simulate(vm["simulation_time"].as<int>());
		return 0;
	}
	else if (vm["scenario"].as<string>() == "ONLINE")
	{
		OnlineGrid G;
		if (!G.load_map(vm["map"].as<std::string>()))
			return -1;
		MAPFSolver* solver = set_solver(G, vm);
		OnlineSystem system(G, *solver);
		assert(!system.hold_endpoints);
		assert(!system.useDummyPaths);
		set_parameters(system, vm);
		G.preprocessing(system.consider_rotation);
		system.simulate(vm["simulation_time"].as<int>());
		return 0;
	}
	else
	{
		cout << "Scenario " << vm["scenario"].as<string>() << "does not exist!" << endl;
		return -1;
	}
}

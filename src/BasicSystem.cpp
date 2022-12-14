#include "BasicSystem.h"
#include <stdlib.h>
#include <boost/tokenizer.hpp>


BasicSystem::BasicSystem(const BasicGraph& G, MAPFSolver& solver): G(G), solver(solver), num_of_tasks(0) {}

BasicSystem::~BasicSystem() {}


bool BasicSystem::load_locations()
{
	string fname = G.map_name + "_rotation=" + std::to_string(consider_rotation) +
		"_" + std::to_string(num_of_drives) + ".agents";
    std::ifstream myfile (fname.c_str());
    if (!myfile.is_open())
		return false;

    string line;
    getline (myfile,line);
    boost::char_separator<char> sep(",");

    if (atoi(line.c_str()) != num_of_drives)
    {
        cout << "The agent file does not match the settings." << endl;
        exit(-1);
    }
    for (int k = 0; k < num_of_drives; k++)
    {
        getline (myfile, line);
        boost::tokenizer< boost::char_separator<char> > tok(line, sep);
        boost::tokenizer< boost::char_separator<char> >::iterator beg=tok.begin();
        // starts
        int start_loc = atoi((*beg).c_str());
        beg++;
        int start_orient = atoi((*beg).c_str());
        beg++;
        starts[k] = State(start_loc, 0, start_orient);
        paths[k].emplace_back(starts[k]);
        finished_tasks[k].push_back(std::make_pair(start_loc, 0));
        // goals
        int goal = atoi((*beg).c_str());
        goal_locations[k].emplace_back(goal, 0);
    }
    myfile.close();
	return true;
}


void BasicSystem::update_start_locations()
{
    for (int k = 0; k < num_of_drives; k++)
    {
        starts[k] = State(paths[k][timestep].location, 0, paths[k][timestep].orientation);
    }
}


void BasicSystem::update_paths(const std::vector<Path*>& MAPF_paths, int max_timestep = INT_MAX)
{
    for (int k = 0; k < num_of_drives; k++)
    {
        int length = min(max_timestep, (int) MAPF_paths[k]->size());
        paths[k].resize(timestep + length);
        for (int t = 0; t < length; t++)
        {
            if (MAPF_paths[k]->at(t).location < 0)
                paths[k][timestep + t] = State(-starts[k].location - 1, timestep + t, starts[k].orientation);
            else
            {
                paths[k][timestep + t] = MAPF_paths[k]->at(t);
                paths[k][timestep + t].timestep = timestep + t;
            }
        }
    }
}

void BasicSystem::update_paths(const std::vector<Path>& MAPF_paths, int max_timestep = INT_MAX)
{
    for (int k = 0; k < num_of_drives; k++)
    {
        int length = min(max_timestep, (int) MAPF_paths[k].size());
        paths[k].resize(timestep + length);
        for (int t = 0; t < length; t++)
        {
            paths[k][timestep + t] = MAPF_paths[k][t];
            paths[k][timestep + t].timestep = timestep + t;
        }
    }
}

void BasicSystem::update_initial_paths(vector<Path>& initial_paths) const
{
    initial_paths.clear();
    initial_paths.resize(num_of_drives);
    for (int k = 0; k < num_of_drives; k++)
    {
        // check whether the path traverse every goal locations
        int i = (int)goal_locations[k].size() - 1;
        int j = (int)paths[k].size() - 1;
        while (i >= 0 && j >= 0)
        {
            while (j >= 0 && paths[k][j].location != goal_locations[k][i].first &&
			paths[k][j].timestep >= goal_locations[k][i].second)
                j--;
            i--;
        }
        if (j < 0)
            continue;
          
        if ((int) paths[k].size() <= timestep + planning_window)
            continue;

        initial_paths[k].resize(paths[k].size() - timestep);
        for (int t = 0; t < (int)initial_paths[k].size(); t++)
        {
            initial_paths[k][t] = paths[k][timestep + t];
            initial_paths[k][t].timestep = t;
        }
    }
}

void BasicSystem::update_initial_constraints(list< tuple<int, int, int> >& initial_constraints) const
{
    initial_constraints.clear();
    for (int k = 0; k < num_of_drives; k++)
    {
        int prev_location = -1;
        for (int t = timestep; t > max(0, timestep - k_robust); t--)
        {
            int curr_location = paths[k][t].location;
            if (curr_location < 0)
                continue;
            else if (curr_location != prev_location)
            {
                initial_constraints.emplace_back(k, curr_location, t + k_robust + 1 - timestep);
                prev_location = curr_location;
            }
        }
    }
}


bool BasicSystem::check_collisions(const vector<Path>& input_paths) const
{
	for (int a1 = 0; a1 < (int)input_paths.size(); a1++)
	{
		for (int a2 = a1 + 1; a2 < (int)input_paths.size(); a2++)
		{
			// TODO: add k-robust
			size_t min_path_length = input_paths[a1].size() < input_paths[a2].size() ? input_paths[a1].size() : input_paths[a2].size();
			for (size_t timestep = 0; timestep < min_path_length; timestep++)
			{
				int loc1 = input_paths[a1].at(timestep).location;
				int loc2 = input_paths[a2].at(timestep).location;
				if (loc1 == loc2)
					return true;
				else if (timestep < min_path_length - 1
					&& loc1 == input_paths[a2].at(timestep + 1).location
					&& loc2 == input_paths[a1].at(timestep + 1).location)
					return true;
			}
			if ((hold_endpoints || useDummyPaths) && input_paths[a1].size() != input_paths[a2].size())
			{
				int a1_ = input_paths[a1].size() < input_paths[a2].size() ? a1 : a2;
				int a2_ = input_paths[a1].size() < input_paths[a2].size() ? a2 : a1;
				int loc1 = input_paths[a1_].back().location;
				for (size_t timestep = min_path_length; timestep < input_paths[a2_].size(); timestep++)
				{
					int loc2 = input_paths[a2_].at(timestep).location;
					if (loc1 == loc2)
						return true;
				}
			}
		}
	}
	return false;
}

bool BasicSystem::congested() const
{
	if (simulation_window <= 1)
		return false;
    int wait_agents = 0;
    for (const auto& path : paths)
    {
        int t = 0;
        while (t < simulation_window && path[timestep].location == path[timestep + t].location &&
                path[timestep].orientation == path[timestep + t].orientation)
            t++;
        if (t == simulation_window)
            wait_agents++;
    }
    return wait_agents > num_of_drives / 2;  // more than half of drives didn't make progress
}

// move all agents from start_timestep to end_timestep
// return a list of finished tasks
list<tuple<int, int, int>> BasicSystem::move()
{
    int start_timestep = timestep;
    int end_timestep = timestep + simulation_window;

	list<tuple<int, int, int>> finished_tasks; // <agent_id, location, timestep>

    for (int t = start_timestep; t <= end_timestep; t++)
    {
        for (int k = 0; k < num_of_drives; k++) {
            // Agents waits at its current locations if no future paths are assigned
            while ((int) paths[k].size() <= t)
            { // This should not happen?
                State final_state = paths[k].back();
                paths[k].emplace_back(final_state.location, final_state.timestep + 1, final_state.orientation);
            }
        }
    }


    for (int t = start_timestep; t <= end_timestep; t++)
    {
        for (int k = 0; k < num_of_drives; k++)
        {
            State curr = paths[k][t];

            /* int wait_times = 0; // wait time at the current location
            while (wait_times < t && paths[k][t - wait_times] != curr)
            {
                wait_times++;
            }*/

            // remove goals if necessary
            if ((!hold_endpoints || paths[k].size() == t + 1) && !goal_locations[k].empty() && 
				curr.location == goal_locations[k].front().first &&
				curr.timestep >= goal_locations[k].front().second) // the agent finish its current task
            {
                goal_locations[k].erase(goal_locations[k].begin());
				finished_tasks.emplace_back(k, curr.location, t);
            }

            // check whether the move is valid
            if (t > 0)
            {
                State prev = paths[k][t - 1];

                if (curr.location == prev.location)
                {
                    if (G.get_rotate_degree(prev.orientation, curr.orientation) == 2)
                    {
                        cout << "Drive " << k << " rotates 180 degrees from " << prev << " to " << curr << endl;
                        save_results();
                        exit(-1);
                    }
                }
                else if (consider_rotation)
                {
                    if (prev.orientation != curr.orientation)
					{
						cout << "Drive " << k << " rotates while moving from " << prev << " to " << curr << endl;
						save_results();
						exit(-1);
					}
					else if ( !G.valid_move(prev.location, prev.orientation) ||
                        prev.location + G.move[prev.orientation] != curr.location)
                    {
                        cout << "Drive " << k << " jump from " << prev << " to " << curr << endl;
                        save_results();
                        exit(-1);
                    }
                }
				else
				{
					int dir = G.get_direction(prev.location, curr.location);
					if (dir < 0 || !G.valid_move(prev.location, dir))
					{
						cout << "Drive " << k << " jump from " << prev << " to " << curr << endl;
						save_results();
						exit(-1);
					}
				}
            }

            // Check whether this move has conflicts with other agents
			if (G.types[curr.location] != "Magic")
			{
				for (int j = k + 1; j < num_of_drives; j++)
				{
					for (int i = max(0, t - k_robust); i <= min(t + k_robust, end_timestep); i++)
					{
						if ((int)paths[j].size() <= i)
							break;
						if (paths[j][i].location == curr.location)
						{
							cout << "Drive " << k << " at " << curr << " has a conflict with drive " << j
								<< " at " << paths[j][i] << endl;
							save_results(); //TODO: write termination reason to files
							exit(-1);
						}
					}
				}
			}
        }
    }
    return finished_tasks;
}


void BasicSystem::save_results()
{
	if (screen)
		std::cout << "*** Saving " << seed << " ***" << std::endl;
    clock_t t = std::clock();
    std::ofstream output;

    // settings
    output.open(outfile + "/config.txt", std::ios::out);
    output << "map: " << G.map_name << std::endl
        << "#drives: " << num_of_drives << std::endl
        << "seed: " << seed << std::endl
        << "solver: " << solver.get_name() << std::endl
        << "time_limit: " << time_limit << std::endl
        << "simulation_window: " << simulation_window << std::endl
        << "planning_window: " << planning_window << std::endl
        << "simulation_time: " << simulation_time << std::endl
        << "robust: " << k_robust << std::endl
        << "rotate: " << consider_rotation << std::endl
        << "use_dummy_paths: " << useDummyPaths << std::endl
        << "hold_endpoints: " << hold_endpoints << std::endl;

    output.close();

    // tasks
    output.open(outfile + "\\tasks.txt", std::ios::out);
    output << num_of_drives << std::endl;
    for (int k = 0; k < num_of_drives; k++)
    {
        int prev = finished_tasks[k].front().first;
        for (auto task : finished_tasks[k])
        {
            output << task.first << "," << task.second << ",";
            if (task.second != 0)
                output << G.heuristics.at(task.first)[prev];
            output << ";";
            prev = task.first;
        }
        for (auto goal : goal_locations[k]) // tasks that have not been finished yet
        {
            output << goal.first << ",-1,;";
        }
        output << std::endl;
    }
    output.close();

    // paths
    output.open(outfile + "\\paths.txt", std::ios::out);
    output << num_of_drives << std::endl;
    for (int k = 0; k < num_of_drives; k++)
    {
        for (auto p : paths[k])
        {
            if (p.timestep <= timestep)
                output << p << ";";
        }
        output << std::endl;
    }
    output.close();
    saving_time = (std::clock() - t) / CLOCKS_PER_SEC;
	if (screen)
		std::cout << "Done! (" << saving_time << " s)" << std::endl;
}


void BasicSystem::update_travel_times(unordered_map<int, double>& travel_times)
{
    if (travel_time_window <= 0)
        return;

    travel_times.clear();
    unordered_map<int, int> count;

    int t_min = max(0, timestep - travel_time_window);
    if (t_min >= timestep)
        return;
    for (auto path : paths)
    {
        int t = timestep;
        while (t >= t_min)
        {
            int loc = path[t].location;
            int dir = path[t].orientation;
            int wait = 0;
            while (t > wait && path[t - 1 - wait].location == loc && path[t - 1 - wait].orientation == dir)
                wait++;
            auto it = travel_times.find(loc);
            if (it == travel_times.end())
            {
                travel_times[loc] = wait;
                count[loc] = 1;
            }
            else
            {
                travel_times[loc] += wait;
                count[loc] += 1;
            }
            t = t - 1 - wait;
        }
    }

    for (auto it : count)
    {
        if (it.second > 1)
        {
            travel_times[it.first] /= it.second;
        }
    }
}


void BasicSystem::solve()
{
    LRA_called = false;
	LRAStar lra(G, solver.path_planner);
	lra.simulation_window = simulation_window;
	lra.k_robust = k_robust;
	solver.clear();
	if (solver.get_name() == "LRA")
	{
		// predict travel time
		unordered_map<int, double> travel_times;
		update_travel_times(solver.travel_times);

		bool sol = solver.run(starts, goal_locations, time_limit);
		update_paths(solver.solution);
	}
	else if (solver.get_name() == "WHCA")
	{
		update_initial_constraints(solver.initial_constraints);

		bool sol = solver.run(starts, goal_locations, time_limit);
		if (sol)
		{
			update_paths(solver.solution);
		}
		else
		{
			lra.resolve_conflicts(solver.solution);
			update_paths(lra.solution);
		}
	}
	 solver.save_results(outfile + "/solver.csv", std::to_string(timestep) + "," 
										+ std::to_string(num_of_drives) + "," + std::to_string(seed));
}


void BasicSystem::initialize_solvers()
{
	solver.k_robust = k_robust;
	solver.window = planning_window;
	solver.hold_endpoints = hold_endpoints || useDummyPaths;
	solver.screen = screen;

	solver.initial_rt.hold_endpoints = true;
	solver.initial_rt.map_size = G.size();
	solver.initial_rt.k_robust = k_robust;
	solver.initial_rt.window = INT_MAX;
}

bool BasicSystem::load_records()
{
	boost::char_separator<char> sep1(";");
	boost::char_separator<char> sep2(",");
	string line;

	// load paths
	std::ifstream myfile(outfile + "/paths.txt");

	if (!myfile.is_open())
			return false;

	timestep = INT_MAX;
	getline(myfile, line);
	if (atoi(line.c_str()) != num_of_drives)
	{
		cout << "The path file does not match the settings." << endl;
		exit(-1);
	}
	for (int k = 0; k < num_of_drives; k++)
	{
		getline(myfile, line);
		boost::tokenizer< boost::char_separator<char> > tok1(line, sep1);
		for (auto task : tok1)
		{
			boost::tokenizer< boost::char_separator<char> > tok2(task, sep2);
			boost::tokenizer< boost::char_separator<char> >::iterator beg = tok2.begin();
			int loc = atoi((*beg).c_str());
			beg++;
			int orientation = atoi((*beg).c_str());
			beg++;
			int time = atoi((*beg).c_str());
			paths[k].emplace_back(loc, time, orientation);
		}
		timestep = min(timestep, paths[k].back().timestep);
	}
	myfile.close();

	// pick the timestep
	timestep = int((timestep - 1) / simulation_window) * simulation_window; //int((timestep - 1) / simulation_window) * simulation_window;

	// load tasks
	myfile.open(outfile + "/tasks.txt");
	if (!myfile.is_open())
		return false;

	getline(myfile, line);
	if (atoi(line.c_str()) != num_of_drives)
	{
		cout << "The task file does not match the settings." << endl;
		exit(-1);
	}
	for (int k = 0; k < num_of_drives; k++)
	{
		getline(myfile, line);
		boost::tokenizer< boost::char_separator<char> > tok1(line, sep1);
		for (auto task : tok1)
		{
			boost::tokenizer< boost::char_separator<char> > tok2(task, sep2);
			boost::tokenizer< boost::char_separator<char> >::iterator beg = tok2.begin();
			int loc = atoi((*beg).c_str());
			beg++;
			int time = atoi((*beg).c_str());
			if (time >= 0 && time <= timestep)
			{
				finished_tasks[k].emplace_back(loc, time);
				timestep = max(timestep, time);
			}
			else
			{
				goal_locations[k].emplace_back(loc, 0);
			}
		}
	}
	myfile.close();
	return true;
}


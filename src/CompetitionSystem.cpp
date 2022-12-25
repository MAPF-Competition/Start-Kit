#include "CompetitionSystem.h"
#include<boost/tokenizer.hpp>

CompetitionSystem::CompetitionSystem(MAPFPlanner* solver): planner(solver), env(solver->env){
};


bool CompetitionSystem::load_map(string fname){
    std::string line;
    std::ifstream myfile ((fname).c_str());
	if (!myfile.is_open())
    {
	    std::cout << "Map file " << fname << " does not exist. " << std::endl;
        return false;
    }
	
    std::cout << "*** Loading map ***" << std::endl;
    clock_t t = std::clock();
	std::size_t pos = fname.rfind('.');      // position of the file extension
  map_name = fname.substr(0, pos);     // get the name without extension
  getline (myfile, line); 
	boost::char_separator<char> sep(",");
	boost::tokenizer< boost::char_separator<char> > tok(line, sep);
	boost::tokenizer< boost::char_separator<char> >::iterator beg = tok.begin();
	rows = atoi((*beg).c_str()); // read number of rows
	beg++;
	cols = atoi((*beg).c_str()); // read number of cols

  map.resize(rows, vector<int>(cols, 0));

	//DeliverGoal.resize(row*col, false);
	// read map
	//int ep = 0, ag = 0;
	for (int i = 0; i < rows; i++)
	{
		getline(myfile, line);
		for (int j = 0; j < cols; j++)
		{
			if (line[j] == '@') // obstacle
			{
				map[i][j] = 1;
			}
			else
			{
				map[i][j] = 0;
			}
		}
	}

	myfile.close();
  double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
  std::cout << "Map size: " << rows << "x" << cols << " with ";
  std::cout << "Done! (" << runtime << " s)" << std::endl;
  return true;
}

bool CompetitionSystem::load_agent_tasks(string fname){
	using namespace std;
	using namespace boost;
  

	string line;
	ifstream myfile(fname.c_str());
	if (!myfile.is_open())
		return false;

	getline(myfile, line);
  while (!myfile.eof() && line[0] == '#'){
    getline(myfile, line);
  }

  char_separator<char> sep(",");
  tokenizer<char_separator<char>> tok(line, sep);
  tokenizer<char_separator<char>>::iterator beg = tok.begin();

  int num_of_agents = atoi((*beg).c_str());

  // My benchmark
  if (num_of_agents == 0)
		{
			cerr << "The number of agents should be larger than 0" << endl;
			exit(-1);
		}
  starts.resize(num_of_agents);
  task_queue.resize(num_of_agents);
  for (int i = 0; i < num_of_agents; i++)
		{
      cout << "agent " << i << ": ";

			getline(myfile, line);
      while (!myfile.eof() && line[0] == '#'){
        getline(myfile, line);
      }
			tokenizer<char_separator<char>> tok(line, sep);
			tokenizer<char_separator<char>>::iterator beg = tok.begin();
			// read start [row,col] for agent i
			int num_landmarks = atoi((*beg).c_str());
      beg++;
      auto loc = atoi((*beg).c_str());
      // agent_start_locations[i] = {loc, 0};
      starts[i] = State(loc, 0, 0);
      cout << loc;
      beg++;
      for (int j = 0; j < num_landmarks; j++, beg++){
        auto loc = atoi((*beg).c_str());
        task_queue[i].push_back(loc);
        cout << " -> " << loc;
      }
      cout << endl;

		}

  myfile.close();
	return true;

}


list<tuple<int, int, int>> CompetitionSystem::move(vector<State>& next_states){
	list<tuple<int, int, int>> finished_tasks; // <agent_id, location, timestep>
  if (valid_moves(curr_states, next_states)){
    curr_states = next_states;
    // check finished tasks;
  }
  // agents do not move

  for (int k = 0; k < num_of_agents; k++) {
    if (curr_states[k].location == goal_locations[k].front().first){
      goal_locations[k].erase(goal_locations[k].begin());
      finished_tasks.emplace_back(k, curr_states[k].location, timestep + 1);
    }

    paths[k].push_back(curr_states[k]);
  }


  return finished_tasks;
}


bool CompetitionSystem::valid_moves(vector<State>& prev, vector<State> next){
  // TODO
  return true;
}

void CompetitionSystem::sync_shared_env(){
  env->goal_locations = goal_locations;
  env->curr_timestep = timestep;
  env->curr_states = curr_states;
}



void CompetitionSystem::simulate(int simulation_time){
	initialize();

	for (; timestep < simulation_time; timestep += 1)
    {
      std::cout << "Timestep " << timestep << std::endl;


      // solve();

      sync_shared_env();

      auto next_states = planner->plan(plan_time_limit);

      // move drives
      auto new_finished_tasks = move(next_states);
      std::cout << new_finished_tasks.size() << " tasks has been finished" << std::endl;

      // update tasks
      for (auto task : new_finished_tasks)
        {
          int id, loc, t;
          std::tie(id, loc, t) = task;
          finished_tasks[id].emplace_back(loc, t);
          // num_of_tasks++;
        }

      update_goal_locations();
    }

	std::cout << std::endl << "Done!" << std::endl;
}



void CompetitionSystem::initialize(){
	// starts.resize(num_of_agents);
	// goal_locations.resize(num_of_agents);
  // task_queue.resize(num_of_drives);

	paths.resize(num_of_agents);
  env->num_of_agents = num_of_agents;
  env->map = map;
	finished_tasks.resize(num_of_agents);
	// bool succ = load_records(); // continue simulating from the records
  timestep = 0;
  curr_states = starts;
  // initialize_goal_locations();
  update_goal_locations();

  sync_shared_env();
  planner->initialize(preprocess_time_limit);
}


void CompetitionSystem::update_goal_locations(){
	for (int k = 0; k < num_of_agents; k++)
    {
      if (goal_locations[k].empty() && !task_queue[k].empty()){
        goal_locations[k].emplace_back(task_queue[k].front(), timestep);
        task_queue[k].pop_front();
      }
    }
}

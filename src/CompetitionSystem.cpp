#include <cmath>
#include "CompetitionSystem.h"
#include <boost/tokenizer.hpp>


list<tuple<int, int, int>> BaseSystem::move(vector<State>& next_states){
	list<tuple<int, int, int>> finished_tasks; // <agent_id, location, timestep>
    if (valid_moves(curr_states, next_states)){
        curr_states = next_states;
        // check finished tasks;
    }
    // agents do not move
    for (int k = 0; k < num_of_agents; k++) {
        if (!goal_locations[k].empty() && curr_states[k].location == goal_locations[k].front().first){
            goal_locations[k].erase(goal_locations[k].begin());
            finished_tasks.emplace_back(k, curr_states[k].location, timestep + 1);
        }
        paths[k].push_back(curr_states[k]);
        planner_movements[k].push_back(next_states[k]);
        actual_movements[k].push_back(curr_states[k]);
    }

    return finished_tasks;
}


// This function might not work correctly with small map (w or h <=2)
bool BaseSystem::valid_moves(vector<State>& prev, vector<State>& next){
  if (validator != nullptr){
    return validator->is_valid(prev, next);
  } else {
    return true;
  }

}


void BaseSystem::sync_shared_env(){
    env->goal_locations = goal_locations;
    env->curr_timestep = timestep;
    env->curr_states = curr_states;
}


void BaseSystem::simulate(int simulation_time){
	initialize();
    int num_of_tasks = 0;
    //I just put it out to seperate ours initilize with participants'
    planner->initialize(preprocess_time_limit);
	for (; timestep < simulation_time; timestep += 1) {
        cout << "----------------------------" << std::endl;
        cout << "Timestep " << timestep << std::endl;

        // find a plan
        sync_shared_env();
        vector<State> next_states = planner->plan(plan_time_limit);

        // move drives
        list<tuple<int, int, int>> new_finished_tasks = move(next_states);
        cout << new_finished_tasks.size() << " tasks has been finished in this timestep" << std::endl;

        // update tasks
        for (auto task : new_finished_tasks) {
            int id, loc, t;
            std::tie(id, loc, t) = task;
            finished_tasks[id].emplace_back(loc, t);
            num_of_tasks++;
        }
        cout << num_of_tasks << " tasks has been finished by far in total" << std::endl;

        update_goal_locations();

        bool complete_all = false;
        for (auto t: goal_locations)
        {
          if(t.empty())
            complete_all = true;
          else
          {
            complete_all = false;
            break;
          }
        }
        if (complete_all)
        {
          cout << std::endl << "All task finished!" << std::endl;
          break;
        }
    }

	cout << std::endl << "Done!" << std::endl;
}


void BaseSystem::initialize() {
	// starts.resize(num_of_agents);
	// goal_locations.resize(num_of_agents);
    // task_queue.resize(num_of_drives);

	paths.resize(num_of_agents);
    env->num_of_agents = num_of_agents;
    env->rows = map.rows;
    env->cols = map.cols;
    env->map = map.map;
	finished_tasks.resize(num_of_agents);
	// bool succ = load_records(); // continue simulating from the records
    timestep = 0;
    curr_states = starts;
    goal_locations.resize(num_of_agents);
    // initialize_goal_locations();
    update_goal_locations();

    sync_shared_env();

    actual_movements.resize(num_of_agents);
    planner_movements.resize(num_of_agents);
    for (int i = 0; i < num_of_agents; i++)
    {
      actual_movements[i].push_back(curr_states[i]);
      planner_movements[i].push_back(curr_states[i]);
    }
    //planner->initialize(preprocess_time_limit);
}

void BaseSystem::saveErrors(const string &fileName) const
{
  std::ofstream output;
  output.open(fileName, std::ios::out);
  // for (int i = 0; i < num_of_agents; i++)
  //   {
  //     output << "Agent " << i << ": ";
  //     if (option == 0)
  //       {
  //         for (const auto t : actual_movements[i])
  //           // output << "(" << t.location
  //           output << "(" << t.location / map.cols << "," << t.location % map.cols
  //                  << "," << t.orientation << ")->";
  //       }
  //     else if (option == 1)
  //       {
  //         for (const auto t : planner_movements[i])
  //           // output << "(" << t.location
  //           output << "(" << t.location / map.cols << "," << t.location % map.cols
  //                  << "," << t.orientation << ")->";
  //       }
  //     output << endl;
  //   }
  for (auto error: validator->errors)
  {
    std::string error_msg;
    int agent1;
    int agent2;
    int timestep;

    std::tie(error_msg,agent1,agent2,timestep) = error;

    output<<"("<<agent1<<","<<agent2<<","<<timestep<<",\""<< error_msg <<"\")"<<endl;
  }
  output.close();
}

void BaseSystem::savePaths(const string &fileName, int option) const
{
  std::ofstream output;
  output.open(fileName, std::ios::out);
  for (int i = 0; i < num_of_agents; i++)
    {
      output << "Agent " << i << ": ";
      if (option == 0)
        {
          for (const auto t : actual_movements[i])
            // output << "(" << t.location
            output << "(" << t.location / map.cols << "," << t.location % map.cols
                   << "," << t.orientation << ")->";
        }
      else if (option == 1)
        {
          for (const auto t : planner_movements[i])
            // output << "(" << t.location
            output << "(" << t.location / map.cols << "," << t.location % map.cols
                   << "," << t.orientation << ")->";
        }
      output << endl;
    }
  output.close();
}

bool FixedAssignSystem::load_agent_tasks(string fname){
	string line;
	std::ifstream myfile(fname.c_str());
	if (!myfile.is_open()) return false;

	getline(myfile, line);
    while (!myfile.eof() && line[0] == '#') {
        getline(myfile, line);
    }

    boost::char_separator<char> sep(",");
    boost::tokenizer<boost::char_separator<char>> tok(line, sep);
    boost::tokenizer<boost::char_separator<char>>::iterator beg = tok.begin();

    num_of_agents = atoi((*beg).c_str());

    // My benchmark
    if (num_of_agents == 0) {
        std::cerr << "The number of agents should be larger than 0" << endl;
        exit(-1);
    }
    starts.resize(num_of_agents);
    task_queue.resize(num_of_agents);
    for (int i = 0; i < num_of_agents; i++) {
        cout << "agent " << i << ": ";

        getline(myfile, line);
        while (!myfile.eof() && line[0] == '#'){
            getline(myfile, line);
        }
        boost::tokenizer<boost::char_separator<char>> tok(line, sep);
        boost::tokenizer<boost::char_separator<char>>::iterator beg = tok.begin();
        // read start [row,col] for agent i
        int num_landmarks = atoi((*beg).c_str());
        beg++;
        auto loc = atoi((*beg).c_str());
        // agent_start_locations[i] = {loc, 0};
        starts[i] = State(loc, 0, 0);
        cout << loc;
        beg++;
        for (int j = 0; j < num_landmarks; j++, beg++) {
            auto loc = atoi((*beg).c_str());
            task_queue[i].push_back(loc);
            cout << " -> " << loc;
        }
        cout << endl;

    }
    myfile.close();

	return true;
}


void FixedAssignSystem::update_goal_locations(){
	for (int k = 0; k < num_of_agents; k++) {
        if (goal_locations[k].empty() && !task_queue[k].empty()) {
            goal_locations[k].emplace_back(task_queue[k].front(), timestep);
            task_queue[k].pop_front();
        }
    }
}


// void TaskAssignSystem::update_goal_locations(){
// 	for (int k = 0; k < num_of_agents; k++) {
//     if (goal_locations[k].empty() && !task_queue.empty()) {
//       std::cout << "assigned task " << task_queue.front() << " to agent " << k << std::endl;
//       goal_locations[k].emplace_back(task_queue.front(), timestep);
//       task_queue.pop_front();
//     }
//   }
// }

void TaskAssignSystem::update_goal_locations(){
	for (int k = 0; k < num_of_agents; k++) {
    while (goal_locations[k].size() < num_tasks_reveal)
    {
      if (!task_queue.empty())
      {
        std::cout << "assigned task " << task_queue.front() << " to agent " << k << std::endl;
        goal_locations[k].emplace_back(task_queue.front(), timestep);
        task_queue.pop_front();
      }
    }
  }
}

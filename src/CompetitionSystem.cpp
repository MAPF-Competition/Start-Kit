#include "CompetitionSystem.h"

CompetitionSystem::CompetitionSystem(const CompetitionGrid & G, MAPFSolver& solver): BasicSystem(G, solver), G(G) {
};


void CompetitionSystem::simulate(int simulation_time){
	initialize();

	for (; timestep < simulation_time; timestep += simulation_window)
    {
      std::cout << "Timestep " << timestep << std::endl;

      update_start_locations();
      update_goal_locations();
      solve();

      // move drives
      auto new_finished_tasks = move();
      std::cout << new_finished_tasks.size() << " tasks has been finished" << std::endl;

      // update tasks
      for (auto task : new_finished_tasks)
        {
          int id, loc, t;
          std::tie(id, loc, t) = task;
          finished_tasks[id].emplace_back(loc, t);
          num_of_tasks++;
        }
    }

	update_start_locations();
	std::cout << std::endl << "Done!" << std::endl;
}



void CompetitionSystem::initialize(){
	initialize_solvers();

	starts.resize(num_of_drives);
	goal_locations.resize(num_of_drives);
  task_queue.resize(num_of_drives);

	paths.resize(num_of_drives);
	finished_tasks.resize(num_of_drives);
	bool succ = load_records(); // continue simulating from the records
	if (!succ)
    {
      timestep = 0;
      succ = load_locations();
      if (!succ)
        {
          cout << "Randomly generating initial locations" << endl;
          initialize_start_locations();
          initialize_goal_locations();
        }
    }
}



void CompetitionSystem::initialize_start_locations()
{
	// Choose random start locations
	// Any non-obstacle locations can be start locations
	// Start locations should be unique
	for (int k = 0; k < num_of_drives; k++)
    {
      starts[k] = State(agent_start_locations[k].first, 0, agent_start_locations[k].second);
      paths[k].emplace_back(starts[k]);
      // finished_tasks[k].emplace_back(G.agent_home_locations[k], 0);
    }
}


void CompetitionSystem::initialize_goal_locations()
{
	for (int k = 0; k < num_of_drives; k++)
    {
      if (!task_queue[k].empty()){
        goal_locations[k].emplace_back(task_queue[k].front(), 0);
        task_queue[k].pop_front();
      }
    }
}


void CompetitionSystem::update_goal_locations(){
  // TODO assign new goal while there is one.
}

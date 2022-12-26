#pragma once
// #include "BasicSystem.h"
#include "SharedEnv.h"
#include "MAPFPlanner.h"




class CompetitionSystem 
{
 public:
	CompetitionSystem(MAPFPlanner* solver);
	~CompetitionSystem(){};

  bool load_map(string fname);
  bool load_agent_tasks(string fname);

  bool check_collisions = true;

	void simulate(int simulation_time);

 private:

  int moves[4];

  int rows = 0;
  int cols = 0;
  std::vector<int> map;
  string map_name;

  MAPFPlanner* planner;
  SharedEnvironment* env;


  // #timesteps for simulation
  int timestep;

  int preprocess_time_limit;
  int plan_time_limit;



  std::vector<Path> paths;
  std::vector<std::list<std::pair<int, int> > > finished_tasks; // location + finish time

  vector<State> starts;


  int num_of_agents;

  vector<State> curr_states;

  // vector of <loc, orientation>
  // initialized in load_tasks

  // all tasks that haven't been finished
  vector<deque<int>> task_queue;

  // tasks that haven't been finished but have been revealed to agents;
  vector< vector<pair<int, int> > > goal_locations;

	void initialize();
	void update_goal_locations();

  void sync_shared_env();


  // move agents,  update agents location, return finished tasks
  list<tuple<int, int, int>> move(vector<State>& next_states);

  bool valid_moves(vector<State>& prev, vector<State> next);

};

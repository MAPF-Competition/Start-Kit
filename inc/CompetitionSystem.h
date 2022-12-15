#pragma once
#include "BasicSystem.h"
#include "CompetitionGraph.h"

class CompetitionSystem :
public BasicSystem
{
 public:
	CompetitionSystem(const CompetitionGrid & G, MAPFSolver& solver);
	~CompetitionSystem();

  void load_agent_tasks(string fname);

	void simulate(int simulation_time);

 private:

  // vector of <loc, orientation>
  // initialized in load_tasks
  vector<pair<int, int>> agent_start_locations;


	const CompetitionGrid& G;


  vector<deque<int>> task_queue;


	void initialize();
	void initialize_start_locations();
	void initialize_goal_locations();
	void update_goal_locations();
};

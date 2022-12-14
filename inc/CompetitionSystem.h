#pragma once
#include "BasicSystem.h"
#include "CompetitionGraph.h"

class CompetitionSystem :
public BasicSystem
{
 public:
	CompetitionSystem(const CompetitionGrid & G, MAPFSolver& solver, std::vector<State> start_locs);
	~CompetitionSystem();

	void simulate(int simulation_time);


  void load_tasks(string fname);





 private:
	const CompetitionGrid& G;


	void initialize();
	/* void initialize_start_locations(); */
	/* void initialize_goal_locations(); */
	void update_goal_locations();
};

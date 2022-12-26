#pragma once
#include "States.h"

class SharedEnvironment {
public:
  int num_of_agents;

  int rows;
  int cols;
  std::vector<int> map;


  // goal locations for each agent
  // each task is a pair of <goal_loc, reveal_time>
  vector< vector<pair<int, int> > > goal_locations;

  int curr_timestep = 0;
  vector<State> curr_states;

};

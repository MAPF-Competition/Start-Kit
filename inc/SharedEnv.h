#pragma once
#include "States.h".h"

class SharedEnvironment {
public:
  int num_of_agents;
  int curr_timestep = 0;
  std::vector<vector<int>> map;
  vector< vector<pair<int, int> > > goal_locations;
  vector<State> curr_states;

};

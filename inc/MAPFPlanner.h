#pragma once
#include <ctime>
#include "SharedEnv.h"

class MAPFPlanner
{
public:

  SharedEnvironment* env;

	//MAPFPlanner(SharedEnvironment* env): env(env){};
  MAPFPlanner(){env = new SharedEnvironment();};
	~MAPFPlanner();


  void initialize(int preprocess_time_limit);

  // return next states for all agents
  std::vector<State> plan(int time_limit);

};

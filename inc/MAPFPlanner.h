#pragma once
#include <ctime>
#include "SharedEnv.h"

class Action{

};

class MAPFPlanner
{
public:

  SharedEnvironment* env;

	MAPFPlanner(SharedEnvironment* env): env(env){};
	~MAPFPlanner();


  void initialize(int preprocess_time_limit);

  // return next states for all agents
  std::vector<State> plan(int time_limit);

};

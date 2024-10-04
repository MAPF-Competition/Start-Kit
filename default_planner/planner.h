#ifndef PLANNER
#define PLANNER

#include "Types.h"
#include "TrajLNS.h"
#include <random>


namespace DefaultPlanner{

    
    void initialize(int preprocess_time_limit, SharedEnvironment* env);

    void plan(int time_limit,vector<Action> & actions,  SharedEnvironment* env);


}
#endif
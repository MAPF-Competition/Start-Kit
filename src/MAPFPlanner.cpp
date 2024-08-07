#include <random>
#include <Entry.h>

//default planner includes
#include "planner.h"


void MAPFPlanner::initialize(int preprocess_time_limit)
{
    // use the remaining entry time limit (after task scheduling) for path planning, -10 for timing error tolerance;
    int limit = preprocess_time_limit - std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() - 10;
    TrafficMAPF::initialize(limit, env);
    return;
}


// plan using simple A* that ignores the time dimension
void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{
    // use the remaining time after task schedule for path planning, -10 for timing error tolerance;
    int limit = time_limit - std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() - 10;
    // cap the planner time to half of the total time_limit;
    // limit = std::min(limit, time_limit/2) - 10;
    TrafficMAPF::plan(limit, actions, env);
    // cout <<"time used: " <<  std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() <<endl;;
    return;
}

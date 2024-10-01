#include <random>
#include <Entry.h>

//default planner includes
#include "planner.h"
#include "const.h"


void MAPFPlanner::initialize(int preprocess_time_limit)
{
    // use the remaining entry time limit (after task scheduling) for path planning, -PLANNER_TIMELIMIT_TOLERANCE for timing error tolerance;
    int limit = preprocess_time_limit - std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::initialize(limit, env);
    return;
}


// plan using simple A* that ignores the time dimension
void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{
    // use the remaining time after task schedule for path planning, -PLANNER_TIMELIMIT_TOLERANCE for timing error tolerance;
    int limit = time_limit - std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;

    DefaultPlanner::plan(limit, actions, env);
    return;
}

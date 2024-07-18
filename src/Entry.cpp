#include "Entry.h"

void Entry::initialize(int preprocess_time_limit)
{
    planner->initialize(preprocess_time_limit);
}

void Entry::compute(int time_limit, std::vector<Action> & plan, std::vector<vector<int>> & proposed_schedule)
{
    //first call task schedule
    scheduler->plan(time_limit,proposed_schedule);
    //then update the task location to planner

    //then call planner
    planner->plan(time_limit,plan);
}
#include "TaskScheduler.h"

#include "scheduler.h"

void TaskScheduler::initialize(int preprocess_time_limit)
{
    //give at most half of the entry time_limit to scheduler;
    //-10 for timing error tolerance
    int limit = preprocess_time_limit/2 - 10;
    DefaultPlanner::schedule_initialize(limit, env);    
}

void TaskScheduler::plan(int time_limit, std::vector<int> & proposed_schedule)
{
    //give at most half of the entry time_limit to scheduler;
    //-10 for timing error tolerance
    int limit = time_limit/2 - 10;
    DefaultPlanner::schedule_plan(limit, proposed_schedule, env);
}

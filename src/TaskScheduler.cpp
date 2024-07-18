#include "TaskScheduler.h"

void TaskScheduler::initialize(int preprocess_time_limit)
{
}

void TaskScheduler::plan(int time_limit, std::vector<vector<int>> & proposed_schedule)
{
    proposed_schedule.resize(env->num_of_agents);
    for (int i = 0; i < env->num_of_agents; i++)
    {
        if (env->curr_task_assignment[i].empty())
        {
            proposed_schedule[i].push_back(env->task_pool.front().task_id);
        }
        else
        {
            proposed_schedule[i].push_back(env->curr_task_assignment[i].front());
        }
    }
}
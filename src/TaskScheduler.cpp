#include "TaskScheduler.h"

void TaskScheduler::initialize(int preprocess_time_limit)
{
}

void TaskScheduler::plan(int time_limit, std::vector<int> & proposed_schedule)
{
    proposed_schedule.resize(env->num_of_agents);
    int i_task = 0;
    for (int i = 0; i < env->num_of_agents; i++)
    {
        if (env->curr_task_schedule[i] == -1)
        {
            for (;i_task < env->task_pool.size(); i_task++)
            {
                if (env->task_pool[i_task].agent_assigned == -1)
                {
                    proposed_schedule[i] = env->task_pool[i_task].task_id;
                    env->task_pool[i_task].agent_assigned = i;
                    break;
                }
            }
        }
        else
        {
            proposed_schedule[i] = env->curr_task_schedule[i];
        }
    }
}

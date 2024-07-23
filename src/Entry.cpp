#include "Entry.h"
#include "Tasks.h"

void Entry::initialize(int preprocess_time_limit)
{
    planner->initialize(preprocess_time_limit);
}

void Entry::compute(int time_limit, std::vector<Action> & plan, std::vector<vector<int>> & proposed_schedule)
{
    //first call task schedule
    scheduler->plan(time_limit,proposed_schedule);

    //then update the task location to planner
    for (size_t i = 0; i < proposed_schedule.size(); i++)
    {
        env->goal_locations[i].clear();
        for (int t_id: proposed_schedule[i])
        {
            Task* task_ptr = nullptr;
            for (Task & task: env->task_pool){
                if (task.task_id == t_id){
                    task_ptr = &task;
                    break;
                }
            }
            if(task_ptr == nullptr){continue;}
            for (int i_loc = task_ptr->idx_next_loc; i_loc < task_ptr->locations.size(); i_loc++ )
            {
                env->goal_locations[i].push_back({task_ptr->locations.at(i_loc), task_ptr->t_revealed});
            }
        }
    }

    //then call planner
    planner->plan(time_limit,plan);
}

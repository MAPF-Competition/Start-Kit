#include "Entry.h"
#include "Tasks.h"
#include "utils.h"
#include "heuristics.h"


void Entry::initialize(int preprocess_time_limit)
{
    scheduler->initialize(preprocess_time_limit);
    planner->initialize(preprocess_time_limit);
}

//time_limit in milliseconds
void Entry::compute(int time_limit, std::vector<Action> & plan, std::vector<int> & proposed_schedule)
{
    //first call task schedule
    

    scheduler->plan(time_limit,proposed_schedule);

    //then update the first unfinished errand/location of tasks for planner reference
    update_goal_locations(proposed_schedule);
    
    //then call planner
    planner->plan(time_limit,plan);

}

void Entry::update_goal_locations(std::vector<int> & proposed_schedule)
{
    env->curr_task_schedule = proposed_schedule;
    for (size_t i = 0; i < proposed_schedule.size(); i++)
    {
        env->goal_locations[i].clear();
        int t_id = proposed_schedule[i];
        if (t_id == -1)
            continue;

        int i_loc = env->task_pool[t_id].idx_next_loc;
        env->goal_locations[i].push_back({env->task_pool[t_id].locations.at(i_loc), env->task_pool[t_id].t_revealed});
    }
    return;
}
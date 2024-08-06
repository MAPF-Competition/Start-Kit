#include "Entry.h"
#include "Tasks.h"
#include "utils.h"

void Entry::initialize(int preprocess_time_limit)
{
    planner->initialize(preprocess_time_limit);
}

void Entry::compute(int time_limit, std::vector<Action> & plan, std::vector<int> & proposed_schedule)
{

    // auto manhattanDistance=[](int loc, int loc2,const SharedEnvironment* env){
    //     int loc_x = loc/env->cols;
    //     int loc_y = loc%env->cols;
    //     int loc2_x = loc2/env->cols;
    //     int loc2_y = loc2%env->cols;
    //     return abs(loc_x-loc2_x) + abs(loc_y-loc2_y);
    // };
    // auto get_makespan_of_task=[&](Task &task){
    //     int makespan = 0;
    //     assert(task.locations.size()==2);
    //     for(int i=1;i<task.locations.size();i++)
    //     {
    //         makespan += manhattanDistance(task.locations[i],task.locations[i-1],env);
    //     }
    //     return makespan;
    // };
    // auto compute_median_unallocated_makespan=[&](){
    //     vector<int> unallocated_makespans;
    //     for (Task & task: env->task_pool)
    //     {
    //         if (task.agent_assigned == -1)
    //         {
    //             unallocated_makespans.push_back(get_makespan_of_task(task));
    //         }
    //     }
    //     if (unallocated_makespans.empty())
    //     {
    //         // cout<<"There are no unallocated tasks"<<endl;
    //         return 0;
    //     }
    //     sort(unallocated_makespans.begin(),unallocated_makespans.end());
    //     // std::cout<<"median makespans: "<<unallocated_makespans[unallocated_makespans.size()/2]<<" tasks ize: "<<unallocated_makespans.size()<<" makespan min"<<unallocated_makespans[0]<< "  makespan max"<<unallocated_makespans.back()<<std::endl;
    //     return unallocated_makespans[unallocated_makespans.size()/2];
    // };
    
    // auto median_makespan=compute_median_unallocated_makespan();
    // cout <<env->curr_timestep<<","<<median_makespan<<endl;
    
    //first call task schedule
    scheduler->plan(time_limit,proposed_schedule);

    //then update the task location to planner
    for (size_t i = 0; i < proposed_schedule.size(); i++)
    {
        env->goal_locations[i].clear();
        int t_id = proposed_schedule[i];
        if (t_id == -1)
            continue;
        Task* task_ptr = nullptr;
        for (Task & task: env->task_pool)
        {
            if (task.task_id == t_id){
                task_ptr = &task;
                break;
            }
        }
        if(task_ptr == nullptr){continue;}
        int i_loc = task_ptr->idx_next_loc;
        env->goal_locations[i].push_back({task_ptr->locations.at(i_loc), task_ptr->t_revealed});
    }
    

    //then call planner
    planner->plan(time_limit,plan);
}

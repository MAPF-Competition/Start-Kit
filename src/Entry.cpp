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
    print_task_pool_median_makespan();
    //first call task schedule
    

    scheduler->plan(time_limit,proposed_schedule);

    //then update the first unfinished errand/location of tasks for planner reference
    update_goal_locations(proposed_schedule);
    
    //then call planner
    planner->plan(time_limit,plan);

}

void Entry::update_goal_locations(std::vector<int> & proposed_schedule){
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
    return;
}

void Entry::print_task_pool_median_makespan(){

    auto get_makespan_of_task=[&](Task &task){
        int makespan = 0;
        assert(task.locations.size()==2);
        for(int i=1;i<task.locations.size();i++)
        {
            // makespan += manhattanDistance(task.locations[i],task.locations[i-1],env);
            makespan+=TrafficMAPF::get_h(env,task.locations[i],task.locations[i-1]);

        }
        return makespan;
    };
    auto compute_median_unallocated_makespan=[&](){
        vector<int> unallocated_makespans;
        for (Task & task: env->task_pool)
        {
            if (task.agent_assigned == -1)
            {
                unallocated_makespans.push_back(get_makespan_of_task(task));
            }
        }
        if (unallocated_makespans.empty())
        {
            // cout<<"There are no unallocated tasks"<<endl;
            return 0;
        }
        sort(unallocated_makespans.begin(),unallocated_makespans.end());
        // std::cout<<"median makespans: "<<unallocated_makespans[unallocated_makespans.size()/2]<<" tasks ize: "<<unallocated_makespans.size()<<" makespan min"<<unallocated_makespans[0]<< "  makespan max"<<unallocated_makespans.back()<<std::endl;
        return unallocated_makespans[unallocated_makespans.size()/2];
    };
    
    auto median_makespan=compute_median_unallocated_makespan();
    cout <<env->curr_timestep<<","<<median_makespan<<endl;
}
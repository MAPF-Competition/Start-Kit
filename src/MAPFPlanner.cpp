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


    auto get_makespan_of_task=[&](Task &task){
        int makespan = 0;
        // assert(task.locations.size()==2);
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

    auto compute_max_unallocated_makespan=[&](){
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
        

        return *std::max_element(unallocated_makespans.begin(),unallocated_makespans.end());
    
    };
    
    // auto median_makespan=compute_median_unallocated_makespan();/
    auto max_makespan=compute_max_unallocated_makespan();
    // cout <<env->curr_timestep<<","<<median_makespan<<endl;
    cout <<env->curr_timestep<<","<<max_makespan<<endl;
    // cout <<"time used: " <<  std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() <<endl;;
    return;
}

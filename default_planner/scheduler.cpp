#include "scheduler.h"

namespace DefaultPlanner{

std::mt19937 mt;

void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    // cout<<"schedule initialise limit" << preprocess_time_limit<<endl;
    DefaultPlanner::init_heuristics(env);
    mt.seed(0);
}

void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    proposed_schedule.resize(env->num_of_agents, -1); //default no schedule


    int i_task, min_task_i, min_task_makespan, dist, c_loc, count;
    clock_t start = clock();

    for (int i = 0; i < env->num_of_agents && std::chrono::steady_clock::now() < endtime; i++)
    {
        
        if (env->curr_task_schedule[i] == -1)
        {
            
            min_task_i = -1;
            min_task_makespan = INT_MAX;
            count = 0;
            for (i_task=0 ; i_task < env->task_pool.size() && std::chrono::steady_clock::now() < endtime ;i_task++)
            {                

                if (env->task_pool[i_task].agent_assigned != -1)
                    continue;
                dist = 0;
                c_loc = env->curr_states.at(i).location;
                for (int loc : env->task_pool[i_task].locations){
                    dist += DefaultPlanner::get_h(env, c_loc, loc);
                    c_loc = loc;
                }
                if (dist < min_task_makespan){
                    min_task_i = i_task;
                    min_task_makespan = dist;
                }
                count++;            
            }


            if (min_task_i != -1){
                proposed_schedule[i] = env->task_pool[min_task_i].task_id;
                env->task_pool[min_task_i].agent_assigned = i;
            }
            else{
                proposed_schedule[i] = -1;
            }
            

        }
        else
        {
            proposed_schedule[i] = env->curr_task_schedule[i];
        }
    }
    // cout << ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
}
}

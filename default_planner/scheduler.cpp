#include "scheduler.h"
#include <algorithm>
#include <random>

namespace DefaultPlanner{

std::mt19937 mt;
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;
std::unordered_set<int> abandoned_tasks;


void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    // cout<<"schedule initialise limit" << preprocess_time_limit<<endl;
    DefaultPlanner::init_heuristics(env);
    mt.seed(0);
    return;
}

void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    int min_task_i, min_task_makespan, dist, c_loc, count;
    clock_t start = clock();
    cout<<"num abandoned tasks: "<<abandoned_tasks.size()<<endl;    
    for(auto t_id : abandoned_tasks){
        if (env->task_pool[t_id].agent_assigned != -1){
            cout<<"error: "<< t_id<< " "<<env->task_pool[t_id].agent_assigned<<" is not -1"<<endl;
            _exit(1);
        }
    }
    abandoned_tasks.clear();

    //random destory
    vector<int> schedule_copy;
    schedule_copy.reserve(env->num_of_agents);
    for (int i=0; i<env->curr_task_schedule.size();i++){
        if (env->curr_task_schedule[i] >=0 && env->task_pool[env->curr_task_schedule[i]].idx_next_loc == 0){
            schedule_copy.push_back(i);
        }
    }



    // iterate over the free agents to decide which task to assign to each of them
    std::unordered_set<int>::iterator it = free_agents.begin();
    while (it != free_agents.end())
    {
        //keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);
            
        min_task_i = -1;
        min_task_makespan = INT_MAX;
        count = 0;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            //check for timeout every 10 task evaluations
            if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            dist = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            // update the new minimum makespan
            if (dist < min_task_makespan){
                min_task_i = t_id;
                min_task_makespan = dist;
            }
            count++;            
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
        }
        // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    if (!schedule_copy.empty() && env->curr_timestep%10 == 0){
        //random shuffle task_pool_copy
        auto rng = std::default_random_engine {};
        std::shuffle(std::begin(schedule_copy), std::end(schedule_copy), rng);
        int c = 0;
        for (auto i : schedule_copy){
            if (c >=5)
                break;
            
            free_agents.insert(i);
            if (env->task_pool[env->curr_task_schedule[i]].agent_assigned != i){
                cout<<"error: "<< env->curr_task_schedule[i]<< " "<<env->task_pool[env->curr_task_schedule[i]].agent_assigned<<" "<<i<<endl;
                _exit(1);
            }

            free_tasks.insert(env->curr_task_schedule[i]);
            abandoned_tasks.insert(env->curr_task_schedule[i]);
            
            proposed_schedule[i] = -1;
            c++;
        }
    }


    #ifndef NDEBUG
    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
    #endif
    return;
}
}

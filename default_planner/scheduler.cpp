#include "scheduler.h"
#include <algorithm>
#include <random>

namespace DefaultPlanner{

std::mt19937 mt;
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;
std::unordered_set<int> abandoned_tasks;
std::unordered_map<int, int> reassigned_tasks;
std::unordered_set<int> reassign_agents;
std::unordered_set<int> abandon_task_agents;


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

    //check correctness of abandoned tasks
    cout<<"num abandoned tasks: "<<abandoned_tasks.size()<<endl;    
    for(auto t_id : abandoned_tasks){
        if (env->task_pool[t_id].agent_assigned != -1){
            cout<<"error: "<< t_id<< " "<<env->task_pool[t_id].agent_assigned<<" is not -1"<<endl;
            _exit(1);
        }
    }
    abandoned_tasks.clear();

    //check correctness of reassigned tasks
    cout<<"num reassigned tasks: "<<reassigned_tasks.size()<<endl;
    for(auto item : reassigned_tasks){
        if (env->task_pool[item.first].agent_assigned != item.second){
            cout<<"error reassign: "<< item.first<< " "<<env->task_pool[item.first].agent_assigned<<" "<<item.second<<endl;
            _exit(1);
        }

    }

    //check correctness of agents abandon their tasks and remain schedule -1
    cout<<"num agents abandon tasks keep schedule -1: "<<abandon_task_agents.size()<<endl;
    for(auto item : abandon_task_agents){
        if (env->curr_task_schedule.at(item) != -1) {
            cout<<"error on agent abandon task: "<< item<< " should have schedule -1 but get task "<<env->curr_task_schedule.at(item)<<endl;
            _exit(1);
        }

    }

    abandon_task_agents.clear();
    reassigned_tasks.clear();
    reassign_agents.clear();


    vector<int> schedule_copy;
    schedule_copy.reserve(env->num_of_agents);
    for (int i=0; i<env->curr_task_schedule.size();i++){
        if (env->curr_task_schedule[i] >=0 && env->task_pool[env->curr_task_schedule[i]].idx_next_loc == 0){
            schedule_copy.push_back(i);
        }
    }
    int c = 0;
    //random remove some tasks
    if (!schedule_copy.empty() && env->curr_timestep%2 == 0){
        //random shuffle task_pool_copy
        auto rng = std::default_random_engine {};
        std::shuffle(std::begin(schedule_copy), std::end(schedule_copy), rng);
        
        for (c=0; c<schedule_copy.size() && c <= 5 ; c++){
            int i = schedule_copy[c];

            
            free_agents.insert(i);
            if (env->task_pool[env->curr_task_schedule[i]].agent_assigned != i){
                cout<<"error agent_assigned does not match schedule: "<< env->curr_task_schedule[i]<< " "<<env->task_pool[env->curr_task_schedule[i]].agent_assigned<<" "<<i<<endl;
                _exit(1);
            }
            abandon_task_agents.insert(i);

            free_tasks.insert(env->curr_task_schedule[i]);
            abandoned_tasks.insert(env->curr_task_schedule[i]);
            
            proposed_schedule[i] = -1;
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

        if (abandon_task_agents.find(i) != abandon_task_agents.end() && rand()%2 == 0)
        {
            proposed_schedule[i] = -1;
            it++;
            continue;
        }
        else if (abandon_task_agents.find(i) != abandon_task_agents.end()){
            abandon_task_agents.erase(i);
        }

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

            if (abandoned_tasks.find(min_task_i) != abandoned_tasks.end()){
                abandoned_tasks.erase(min_task_i);
                reassigned_tasks.insert({min_task_i, i});
                reassign_agents.insert(i);
            }
        }
        // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    if (!schedule_copy.empty() && env->curr_timestep%2 == 0){
        //random shuffle task_pool_copy
        auto rng = std::default_random_engine {};
        std::shuffle(std::begin(schedule_copy), std::end(schedule_copy), rng);
        for (; c<schedule_copy.size() && c <= 10 ; c++){
            int i = schedule_copy[c];

            if (reassign_agents.find(i) != reassign_agents.end()){
                continue;
            }
            
            free_agents.insert(i);
            if (env->task_pool[env->curr_task_schedule[i]].agent_assigned != i){
                cout<<"error: "<< env->curr_task_schedule[i]<< " "<<env->task_pool[env->curr_task_schedule[i]].agent_assigned<<" "<<i<<endl;
                _exit(1);
            }

            free_tasks.insert(env->curr_task_schedule[i]);
            abandoned_tasks.insert(env->curr_task_schedule[i]);
            
            proposed_schedule[i] = -1;
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

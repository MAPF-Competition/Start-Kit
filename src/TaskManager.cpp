#include "TaskManager.h"
#include "nlohmann/json.hpp"
using json = nlohmann::ordered_json;
list<Task> TaskManager::check_finished_tasks(vector<State> states, int timestep){
    
    list<Task> finished_tasks_this_timestep; // <agent_id, task_id, timestep>
    for (int k = 0; k < num_of_agents; k++)
        {
            if (!assigned_tasks[k].empty() && states[k].location == assigned_tasks[k].front().get_next_loc())
                {
                    Task task = assigned_tasks[k].front();
                    task.idx_next_loc += 1;

                    if (task.is_finished()){
                        assigned_tasks[k].pop_front();
                        task.t_completed = timestep;
                        finished_tasks_this_timestep.push_back(task);
                        events[k].push_back(make_tuple(task.task_id, timestep,"finished"));
                        // log_event_finished(k, task.task_id, timestep);
                    } else {
              
                    }
                }
        }


    for (auto task : finished_tasks_this_timestep){
        // int id, loc, t;
        // std::tie(id, loc, t) = task;
        finished_tasks[task.agent_assigned].emplace_back(task);
        num_of_task_finish++;
    }

    return finished_tasks_this_timestep;
}


void TaskManager::sync_shared_env(SharedEnvironment* env) {
    for (size_t i = 0; i < num_of_agents; i++)
        {
            env->goal_locations[i].clear();
            for (auto& task: assigned_tasks[i])
                {
                    for (int i_task = task.idx_next_loc; i_task < task.locations.size(); i_task ++ ){
                        env->goal_locations[i].push_back({task.locations.at(i_task), task.t_assigned });
                    }
                }
        }
}

bool TaskManager::update_tasks(int timestep){
    for (int k = 0; k < num_of_agents; k++)
        {
            while (assigned_tasks[k].size() < num_tasks_reveal) 
                {
                    int i = task_counter[k] * num_of_agents + k;
                    int loc = tasks[i%tasks_size];
                    Task task(task_id,loc,timestep,k);
                    assigned_tasks[k].push_back(task);
                    events[k].push_back(make_tuple(task.task_id,timestep,"assigned"));
                    // log_event_assigned(k, task.task_id, timestep);
                    all_tasks.push_back(task);
                    task_id++;
                    task_counter[k]++;
                }
        }


    bool complete_all = false;
    for (auto & t: assigned_tasks)
        {
            if(t.empty()) 
                {
                    complete_all = true;
                }
            else
                {
                    complete_all = false;
                    break;
                }
        }
    return complete_all;

    // for (int k = 0; k < num_of_agents; k++) {
    //     while (assigned_tasks[k].size() < num_tasks_reveal && !task_queue.empty()){
    //         Task task = task_queue.front();
    //         task.t_assigned = timestep;
    //         task.agent_assigned = k;
    //         task_queue.pop_front();
    //         assigned_tasks[k].push_back(task);
    //         events[k].push_back(make_tuple(task.task_id,timestep,"assigned"));
    //         all_tasks.push_back(task);
    //         // log_event_assigned(k, task.task_id, timestep);
    //     }
    // }
}


json TaskManager::to_json(int map_cols) const{
    
    json tasks = json::array();
    for (auto t: all_tasks)
        {
            json task = json::array();
            task.push_back(t.task_id);
            // TODO rewrite the task output part
            task.push_back(t.locations.front()/map_cols);
            task.push_back(t.locations.front()%map_cols);
            tasks.push_back(task);
        }
    return tasks;
}

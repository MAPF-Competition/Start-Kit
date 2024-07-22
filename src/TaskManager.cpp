#include "TaskManager.h"
#include "Tasks.h"
#include "nlohmann/json.hpp"
#include <vector>

using json = nlohmann::ordered_json;


bool TaskManager::validate_task_assgnment(vector< vector<int> > & assignment)
{
    if (assignment.size() != num_of_agents)
    {
        return false;
    }

    unordered_set<int> idx_set;

    for (int i_agent = 0; i_agent < assignment.size(); i_agent ++)
    {
        // if agent is already executing some task, it should be assigned the same task.
        if (assigned_tasks[i_agent].front()->idx_next_loc > 0 &&
            (assignment[i_agent].empty()  || assignment[i_agent].front() != assigned_tasks[i_agent].front()->task_id)
            ){
            return false;
        }

        for (int i_task = 0; i_task < assignment[i_agent].size(); i_task++){
            // task should be a ongoing task
            if (ongoing_tasks.find(assignment[i_agent][i_task]) == ongoing_tasks.end()){
                return false;
            }

            // one task should not appear in the assignment twice
            if (idx_set.find(assignment[i_agent][i_task]) != idx_set.end()){
                return false;
            }

        }
    }

    return true;
}

bool TaskManager::set_task_assignment(vector< vector<int> > & assignment)
{
    if (! validate_task_assgnment(assignment))
    {
        logger->log_warning("attempt to set invalid task assignment");
        return false;
    }

    for (int i = 0; i < num_of_agents; i++)
    {
        assigned_tasks[i].clear();
        for (int i_task; i_task< assignment[i].size(); i ++)
        {
            assigned_tasks[i].push_back(ongoing_tasks.at(assignment[i][i_task]));
        }
    }

    return true;
}

list<int> TaskManager::check_finished_tasks(vector<State> states, int timestep){
    
    list<int> finished_tasks_this_timestep; // <agent_id, task_id, timestep>
    for (int k = 0; k < num_of_agents; k++)
    {
        if (!assigned_tasks[k].empty() && states[k].location == assigned_tasks[k].front()->get_next_loc())
        {
            Task * task = assigned_tasks[k].front();
            task->idx_next_loc += 1;

            if (task->is_finished()){
                assigned_tasks[k].pop_front();
                ongoing_tasks.erase(task->task_id);
                task->t_completed = timestep;

                finished_tasks_this_timestep.push_back(task->task_id);
                events[k].push_back(make_tuple(task->task_id, timestep,"finished"));
                finished_tasks[task->agent_assigned].emplace_back(task);
                num_of_task_finish++;
                // log_event_finished(k, task.task_id, timestep);
            } else {
        
            }
        }
    }
    return finished_tasks_this_timestep;
}


void TaskManager::sync_shared_env(SharedEnvironment* env) 
{
    for (size_t i = 0; i < num_of_agents; i++)
    {
        env->goal_locations[i].clear();
        for (auto& task: assigned_tasks[i])
            {
                for (int i_task = task->idx_next_loc; i_task < task->locations.size(); i_task ++ ){
                    env->goal_locations[i].push_back({task->locations.at(i_task), task->t_assigned });
                }
            }
    }
    for (auto it: ongoing_tasks)
    {
        Task* task_ptr = it.second;
        env->task_pool.push_back(*task_ptr);
    }

    env->curr_task_assignment = vector<vector<int>>(num_of_agents);
    for (int i = 0 ; i < num_of_agents; i++){
        for (Task* task_ptr:assigned_tasks[i]){
            env->curr_task_assignment[i].push_back(task_ptr->task_id);
        }
    }
}

void TaskManager::update_tasks(int timestep)
{
    while (ongoing_tasks.size() < num_tasks_reveal)
    {
        int i = task_id%tasks.size();
        list<int> locs = tasks[i];
        Task* task = new Task(task_id,locs,timestep);
        ongoing_tasks[task->task_id] = task;
        all_tasks.push_back(task);
        task_id++;
    }
}


json TaskManager::to_json(int map_cols) const{
    
    json tasks = json::array();
    for (auto t: all_tasks)
        {
            json task = json::array();
            task.push_back(t->task_id);
            // TODO rewrite the task output part
            task.push_back(t->locations.front()/map_cols);
            task.push_back(t->locations.front()%map_cols);
            tasks.push_back(task);
        }
    return tasks;
}

#include "TaskManager.h"
#include "Tasks.h"
#include "nlohmann/json.hpp"
#include <vector>

using json = nlohmann::ordered_json;


bool TaskManager::validate_task_assgnment(vector< vector<int> > assignment)
{
    if (assignment.size() != num_of_agents)
    {
        return false;
    }

    unordered_set<int> idx_set;

    //here we only check the first assignment to each agent
    for (int i_agent = 0; i_agent < assignment.size(); i_agent ++)
    {
        // task should be a ongoing task
        if (!assignment[i_agent].empty() && ongoing_tasks.find(assignment[i_agent].front()) == ongoing_tasks.end())
        {
            return false;
        }

        // one task should not appear in the assignment twice
        if (!assignment[i_agent].empty() && idx_set.find(assignment[i_agent].front()) != idx_set.end())
        {
            return false;
        }

        // if agent is already executing some task, it should be assigned the same task.
        if (!current_assignment[i_agent].empty()){
            if (ongoing_tasks[current_assignment[i_agent].front()]->idx_next_loc > 0 && (current_assignment[i_agent].empty()  || assignment[i_agent].front() != current_assignment[i_agent].front()))
                {
                    return false;
                }
        }
        if (!assignment[i_agent].empty()){
            idx_set.insert(assignment[i_agent].front());
        }
    }

    return true;
}

bool TaskManager::set_task_assignment(vector< vector<int> > assignment)
{
    if (! validate_task_assgnment(assignment))
    {
        logger->log_warning("attempt to set invalid task assignment");
        return false;
    }

    for (int a = 0; a < assignment.size(); a++)
    {
        current_assignment[a].clear();
        for (int t_id: assignment[a])
        {
            current_assignment[a].push_back(t_id);
            ongoing_tasks[t_id]->agent_assigned = a;
        }
    }

    return true;
}

list<int> TaskManager::check_finished_tasks(vector<State> states, int timestep)
{ 
    list<int> finished_tasks_this_timestep; // <agent_id, task_id, timestep>
    for (int k = 0; k < num_of_agents; k++)
    {
        if (!current_assignment[k].empty() && states[k].location == ongoing_tasks[current_assignment[k].front()]->get_next_loc())
        {
            Task * task = ongoing_tasks[current_assignment[k].front()];
            task->idx_next_loc += 1;

            if (task->is_finished())
            {
                current_assignment[k].erase(current_assignment[k].begin());
                ongoing_tasks.erase(task->task_id);
                task->t_completed = timestep;

                finished_tasks_this_timestep.push_back(task->task_id);
                events[k].push_back(make_tuple(task->task_id, timestep,"finished"));
                finished_tasks[task->agent_assigned].emplace_back(task);
                num_of_task_finish++;
            }
        }
    }
    return finished_tasks_this_timestep;
}


void TaskManager::sync_shared_env(SharedEnvironment* env) 
{
    env->task_pool.clear();
    for (auto it: ongoing_tasks)
    {
        Task* task_ptr = it.second;
        Task temp = new Task(task_ptr);
        env->task_pool.push_back(temp);
    }

    env->curr_task_assignment = current_assignment;

    for (size_t i = 0; i < num_of_agents; i++)
    {
        env->goal_locations[i].clear();
        for (int t_id: current_assignment[i])
        {
            auto& task = ongoing_tasks[t_id];
            for (int i_task = task->idx_next_loc; i_task < task->locations.size(); i_task++ )
            {
                env->goal_locations[i].push_back({task->locations.at(i_task), task->t_revealed});
            }
        }
    }
}

void TaskManager::reveal_tasks(int timestep)
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

void TaskManager::update_tasks(vector<State> states, vector< vector<int> > assignment, int timestep)
{
    set_task_assignment(assignment);
    check_finished_tasks(states,timestep);
    reveal_tasks(timestep);
}


json TaskManager::to_json(int map_cols) const{
    
    json tasks = json::array();
    for (auto t: all_tasks)
    {
        json task = json::array();
        task.push_back(t->task_id);
        // TODO rewrite the task output part
        // task.push_back(t->locations.front()/map_cols);
        // task.push_back(t->locations.front()%map_cols);
        for (auto loc: t->locations)
        {
            task.push_back(loc/map_cols);
            task.push_back(loc%map_cols);
        }
        tasks.push_back(task);
    }
    return tasks;
}

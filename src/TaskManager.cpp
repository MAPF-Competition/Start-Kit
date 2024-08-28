#include "TaskManager.h"
#include "Tasks.h"
#include "nlohmann/json.hpp"
#include <vector>

using json = nlohmann::ordered_json;


bool TaskManager::validate_task_assgnment(vector<int> assignment)
{
    if (assignment.size() != num_of_agents)
    {
        return false;
    }

    unordered_map<int,int> idx_set;

    //here we only check the first assignment to each agent
    for (int i_agent = 0; i_agent < assignment.size(); i_agent ++)
    {
        // task should be a ongoing task
        if (assignment[i_agent] != -1 && ongoing_tasks.find(assignment[i_agent]) == ongoing_tasks.end())
        {
            schedule_errors.push_back(make_tuple("task already finished",assignment[i_agent],i_agent,-1,curr_timestep+1));
            return false;
        }

        // one task should not appear in the assignment twice
        if (assignment[i_agent] != -1 && idx_set.find(assignment[i_agent]) != idx_set.end())
        {
            schedule_errors.push_back(make_tuple("task is already assigned by the second agent at the same time",assignment[i_agent],i_agent,idx_set[assignment[i_agent]],curr_timestep+1));
            return false;
        }

        // if agent is already executing some task, it should be assigned the same task.
        if (current_assignment[i_agent] != -1)
        {
            if (ongoing_tasks[current_assignment[i_agent]]->idx_next_loc > 0 && (current_assignment[i_agent] == -1  || assignment[i_agent] != current_assignment[i_agent]))
            {
                schedule_errors.push_back(make_tuple("task is already opened by the second agent",assignment[i_agent],i_agent,ongoing_tasks[current_assignment[i_agent]]->agent_assigned,curr_timestep+1));
                return false;
            }
        }
        if (assignment[i_agent] != -1)
        {
            idx_set[assignment[i_agent]] = i_agent;
        }
    }

    return true;
}

bool TaskManager::set_task_assignment(vector< int> assignment)
{
    for (int a = 0; a < assignment.size(); a++)
    {
        if (planner_schedule[a].empty() || assignment[a] != planner_schedule[a].back().second)
        {
            planner_schedule[a].push_back(make_pair(curr_timestep,assignment[a]));
        }
    }
    if (! validate_task_assgnment(assignment))
    {
        if (assignment.size() < num_of_agents)
            logger->log_info("task scheduler tiemout");
        else
            logger->log_warning("attempt to set invalid task assignment");
        return false;
    }

    for (int a = 0; a < assignment.size(); a++)
    {
        if (assignment[a] < 0)
            continue;
        int t_id = assignment[a];
        current_assignment[a] = t_id;
        ongoing_tasks[t_id]->agent_assigned = a;
    }

    for (int a = 0; a < current_assignment.size(); a++)
    {
        if (actual_schedule[a].empty() || current_assignment[a] != actual_schedule[a].back().second)
        {
            actual_schedule[a].push_back(make_pair(curr_timestep,current_assignment[a]));
        }
    }

    return true;
}

list<int> TaskManager::check_finished_tasks(vector<State> states, int timestep)
{ 
    list<int> finished_tasks_this_timestep; // <agent_id, task_id, timestep>
    for (int k = 0; k < num_of_agents; k++)
    {
        if (current_assignment[k] != -1 && states[k].location == ongoing_tasks[current_assignment[k]]->get_next_loc())
        {
            Task * task = ongoing_tasks[current_assignment[k]];
            task->idx_next_loc += 1;

            if (task->is_finished())
            {
                current_assignment[k] = -1;
                ongoing_tasks.erase(task->task_id);
                task->t_completed = timestep;

                finished_tasks_this_timestep.push_back(task->task_id);
                finished_tasks[task->agent_assigned].emplace_back(task);
                num_of_task_finish++;
                logger->log_info("Agent " + std::to_string(task->agent_assigned) + " finishes task " + std::to_string(task->task_id), timestep);
                logger->flush();
            }
            events.push_back(make_tuple(timestep,k,task->task_id,task->idx_next_loc));
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
        // env->task_pool[temp.task_id] = temp;
    }

    env->curr_task_schedule = current_assignment;

    for (size_t i = 0; i < num_of_agents; i++)
    {
        env->goal_locations[i].clear();
        int t_id = current_assignment[i];
        if (t_id != -1)
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
        logger->log_info("Task " + std::to_string(task_id) + " is revealed");
    }
}

void TaskManager::update_tasks(vector<State> states, vector<int> assignment, int timestep)
{
    curr_timestep = timestep;
    set_task_assignment(assignment);
    check_finished_tasks(states,timestep);
    reveal_tasks(timestep);
}


json TaskManager::to_json(int map_cols) const
{
    
    json tasks = json::array();
    for (auto t: all_tasks)
    {
        json task = json::array();
        task.push_back(t->task_id);
        // TODO rewrite the task output part
        // task.push_back(t->locations.front()/map_cols);
        // task.push_back(t->locations.front()%map_cols);
        task.push_back(t->t_revealed);
        json locs = json::array();
        for (auto loc: t->locations)
        {
            locs.push_back(loc/map_cols);
            locs.push_back(loc%map_cols);
        }
        task.push_back(locs);
        tasks.push_back(task);
    }
    return tasks;
}

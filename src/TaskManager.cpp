#include "TaskManager.h"
#include "Tasks.h"
#include "nlohmann/json.hpp"
#include <vector>

using json = nlohmann::ordered_json;

/**
 * This function validates the proposed schedule (assignment) from participants
 * 
 * @param assignment a vector of task_ids, one for each agent. The length of the vector should be equal to the number of agents.
 *
 */
bool TaskManager::validate_task_assignment(vector<int>& assignment)
{
    if (assignment.size() != num_of_agents)
    {
        schedule_errors.push_back(make_tuple("Invalid schedule size",-1,-1,-1,curr_timestep+1));
        logger->log_warning("Scheduler Error: assignment size does not match number of agents",curr_timestep+1);
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
            logger->log_warning("Scheduler Error: schedule agent " + std::to_string(i_agent) + " to task " + std::to_string(assignment[i_agent]) + " wrong because the task is already finished",curr_timestep+1);
            logger->flush();
            return false;
        }

        // one task should not appear in the assignment twice
        if (assignment[i_agent] != -1 && idx_set.find(assignment[i_agent]) != idx_set.end())
        {
            schedule_errors.push_back(make_tuple("task is already assigned by the second agent at the same time",assignment[i_agent],i_agent,idx_set[assignment[i_agent]],curr_timestep+1));
            logger->log_warning("Scheduler Error: schedule agent " + std::to_string(i_agent) + " to task " + std::to_string(assignment[i_agent]) + " wrong because the task is already assigned to agent " + std::to_string(idx_set[assignment[i_agent]]),curr_timestep+1);
            return false;
        }

        // if agent is already executing some task, it should be assigned the same task.
        if (current_assignment[i_agent] != -1)
        {
            if (ongoing_tasks[current_assignment[i_agent]]->idx_next_loc > 0 && (current_assignment[i_agent] == -1  || assignment[i_agent] != current_assignment[i_agent]))
            {
                schedule_errors.push_back(make_tuple("task is already opened by the second agent",assignment[i_agent],i_agent,ongoing_tasks[current_assignment[i_agent]]->agent_assigned,curr_timestep+1));
                logger->log_warning("Scheduler Error: schedule agent " + std::to_string(i_agent) + " to task " + std::to_string(assignment[i_agent]) + " wrong because the task is already opened by agent " + std::to_string(ongoing_tasks[current_assignment[i_agent]]->agent_assigned),curr_timestep+1);
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


/**
 * This function updates the current task assignments of agents.
 * It first checks if the proposed assignment is valid, 
 * then updates the current assignment and updates the corresponding agent_assigned of each affected task.
 * 
 * @param assignment a vector of task_ids, one for each agent. The length of the vector should be equal to the number of agents.
 *
 */
bool TaskManager::set_task_assignment(vector< int>& assignment)
{
    for (int a = 0; a < assignment.size(); a++)
    {
        if (planner_schedule[a].empty() || assignment[a] != planner_schedule[a].back().second)
        {
            planner_schedule[a].push_back(make_pair(curr_timestep,assignment[a]));
        }
    }
    if (! validate_task_assignment(assignment))
    {
        return false;
    }

    //reset all the agent_assigned to -1, so that any droped task->agent_assignment will be -1
    for (int a = 0; a < assignment.size(); a++)
    {
        if (current_assignment[a] >= 0){
            ongoing_tasks[current_assignment[a]]->agent_assigned = -1;
        }
    }

    // then set the updated agent_assigned according to new assignments.
    for (int a = 0; a < assignment.size(); a++)
    {
        int t_id = assignment[a];
        current_assignment[a] = t_id;
        if (assignment[a] < 0)
        {
            continue;
        }
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

/**
 * This function checks if any task is finished at the current timestep.
 * If a task is finished, it updates the task's completion time and the agent's current assignment.
 * 
 * @param states a vector of states of all agents, including the current location of each agent on the map.
 * @param timestep the current timestep.
 */
list<int> TaskManager::check_finished_tasks(vector<State>& states, int timestep)
{ 
    list<int> finished_tasks_this_timestep; // <agent_id, task_id, timestep>
    new_freeagents.clear(); //prepare to push all new free agents to the shared environment
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
                new_freeagents.push_back(k); // record the new free agent
                logger->log_info("Agent " + std::to_string(task->agent_assigned) + " finishes task " + std::to_string(task->task_id), timestep);
                logger->flush();
            }
            events.push_back(make_tuple(timestep,k,task->task_id,task->idx_next_loc));
        }
    }
    return finished_tasks_this_timestep;
}

/**
 * This function synchronises the shared environment with the current task manager.
 * It copies the current task pool, current task schedule, new free agents, and new tasks to the shared environment.
 * 
 * @param env a pointer to the shared environment.
 */
void TaskManager::sync_shared_env(SharedEnvironment* env) 
{
    env->task_pool.clear();
    for (auto& task: ongoing_tasks)
    {
        env->task_pool[task.first] = *task.second;
    }
    env->curr_task_schedule = current_assignment;
    env->new_freeagents = new_freeagents;
    env->new_tasks = new_tasks; 
}

/**
 * This function reveals new tasks at the current timestep.
 * It reveals a fixed number of tasks at each timestep, 
 * and adds them to the ongoing tasks, new_tasks, and all_tasks.
 * 
 * @param timestep the current timestep.
 */
void TaskManager::reveal_tasks(int timestep)
{
    new_tasks.clear(); //prepare to push all new revealed tasks to the shared environment
    while (ongoing_tasks.size() < num_tasks_reveal)
    {
        int i = task_id%tasks.size();
        list<int> locs = tasks[i];
        Task* task = new Task(task_id,locs,timestep);
        ongoing_tasks[task->task_id] = task;
        all_tasks.push_back(task);
        new_tasks.push_back(task->task_id);         // record the new tasks
        logger->log_info("Task " + std::to_string(task_id) + " is revealed");
        task_id++;
    }
}

/**
 * This function is reponsible for the task management process:
 * 1. It updates the current assignments of agents with proposed schedule from participants.
 * 2. It checks if any task is finished at the current timestep.
 * 3. It reveals new tasks at the current timestep.
 * 
 * @param states a vector of states of all agents, including the current location of each agent on the map.
 * @param assignment a vector of task_ids, one for each agent. The length of the vector should be equal to the number of agents.
 * @param timestep the current timestep.
 */
void TaskManager::update_tasks(vector<State>& states, vector<int>& assignment, int timestep)
{
    curr_timestep = timestep;
    set_task_assignment(assignment);
    check_finished_tasks(states,timestep);
    reveal_tasks(timestep);
}

/**
 * This function converts all tasks to a JSON object.
 * 
 * @param map_cols the number of columns in the map.
 */
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

#include "Executor.h"

void Executor::initialize(int preprocess_time_limit)
{
   tpg.resize(env->map.size());
   previous_locations.resize(env->num_of_agents, -1);
}

vector<State> Executor::process_new_plan(int sync_time_limit, vector<Action> plan, vector<vector<Action>> & staged_actions)
{
    // Default implementation: always append, update the predicted states based on moves
    vector<State> curr_states = env->system_states;
    vector<State> predicted_states(env->num_of_agents);

    if (env->system_timestep == 0)
    {
        //insert start locations to tpg
        for (int i = 0; i < plan.size(); i++)
        {
            tpg[env->curr_states[i].location].push_back(i);
            previous_locations[i] = env->system_states[i].location;
        }
    }

    int moves[4] = {1, env->cols, -1, -env->cols};
    for (int i = 0; i < plan.size(); i++)
    {
        int new_location = curr_states[i].location;
        int new_orientation = curr_states[i].orientation;
        if (plan[i] == Action::FW)
        {
            new_location = new_location + moves[curr_states[i].orientation];
            tpg[new_location].push_back(i);
        }
        else if (plan[i] == Action::CR)
        {
            new_orientation = (curr_states[i].orientation + 1) % 4;
        }
        else if (plan[i] == Action::CCR)
        {
            new_orientation = (curr_states[i].orientation - 1) % 4;
            if (new_orientation == -1)
                new_orientation = 3;
        }
        predicted_states[i] = State(new_location, curr_states[i].timestep + 1, new_orientation);
        if (plan[i] != Action::NA && plan[i] != Action::W)
        {
            staged_actions[i].push_back(plan[i]);
        }
    }
    return predicted_states;
}

void Executor::next_command(int exec_time_limit, std::vector<vector<Action>> staged_actions, std::vector<ExecutionCommand> & agent_command)
{
    //update the tpg based on the current system states from last tick
    for (int i = 0; i < env->system_states.size(); i++)
    {
        int prev_location = previous_locations[i];
        int curr_location = env->system_states[i].location;
        if (prev_location != curr_location && tpg[prev_location].front() == i)
        {
            //remove the agent from the previous location in tpg
            tpg[prev_location].pop_front();
            previous_locations[i] = curr_location;
        }
    }
    vector<bool> decided(env->num_of_agents, false);
    temp_tpg = tpg; //copy the tpg to temp_tpg for current timestep processing, we will update temp_tpg during mcp but keep tpg unchanged
    for (int i = 0; i < env->system_states.size(); i++)
    {
        // always try to go and clear orders because we don't know the current delay
        if (!decided[i])
            mcp(staged_actions, i, decided, agent_command);
    }
    
}

bool Executor::mcp(std::vector<vector<Action>> staged_actions, int agent_id, vector<bool> & curr_decision, std::vector<ExecutionCommand> & agent_command)
{
    if (staged_actions[agent_id].empty())
    {
        //no action, just stop and wait for the next plan, no tpg order clear
        agent_command[agent_id] = ExecutionCommand::STOP;
        curr_decision[agent_id] = true;
        return false;
    }
    else if (staged_actions[agent_id].front() != Action::FW) 
    {
        //try to go but still stay at current location because it's not move forward, no tpg order clear
        curr_decision[agent_id] = true;
        return false;
    }
    else
    {
        int moves[4] = {1, env->cols, -1, -env->cols};
        int curr_location = env->system_states[agent_id].location;
        int curr_orientation = env->system_states[agent_id].orientation;
        int next_location = curr_location + moves[curr_orientation];
        if (temp_tpg[next_location].empty() || temp_tpg[next_location].front() == agent_id)
        {
            //try to go and can go, clear the tpg order
            agent_command[agent_id] = ExecutionCommand::GO;

            assert(!temp_tpg[curr_location].empty());
            assert(temp_tpg[curr_location].front() == agent_id);

            temp_tpg[curr_location].pop_front();

            curr_decision[agent_id] = true;
            return true;
        }
        else
        {
            //try to go by recursion
            int blocking_agent_id = temp_tpg[next_location].front();
            if (curr_decision[blocking_agent_id])
            {                
                //the blocking agent has already made the decision, so we cannot go
                agent_command[agent_id] = ExecutionCommand::STOP;
                curr_decision[agent_id] = true;
                return false;       
            }
            if (mcp(staged_actions, blocking_agent_id, curr_decision, agent_command))
            {

                if (temp_tpg[curr_location].front() != agent_id)
                {
                    //still more agents on the order, has to wait for the next tick 
                    agent_command[agent_id] = ExecutionCommand::STOP;
                    curr_decision[agent_id] = true;
                    return false;
                }

                //now agent can go and clear the tpg order
                agent_command[agent_id] = ExecutionCommand::GO;
                assert(!temp_tpg[curr_location].empty());

                temp_tpg[curr_location].pop_front();

                curr_decision[agent_id] = true;
                return true;
            }
            else
            {
                //the blocking agent cannot go, so we cannot go
                agent_command[agent_id] = ExecutionCommand::STOP;
                curr_decision[agent_id] = true;
                return false;
            }
        }
    }
    
}

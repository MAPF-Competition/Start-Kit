#include "Executor.h"

void Executor::initialize(int preprocess_time_limit)
{
    tpg.resize(env->map.size());
    previous_locations.resize(env->num_of_agents, -1);
    if (process_plan_type == APPEND_WINDOW)
    {
        window_size = env->min_planner_communication_time / (env->action_time*env->max_counter); //calculate the window size based on the communication time limit and action execution time
        if (window_size < 1)
            window_size = 1;
    }
    else if (process_plan_type == APPEND_ONE)
    {
        window_size = 1;
    }
    else
    {
        window_size = INT_MAX;
    }
    // cout<<"executor initialized with window size: "<<window_size<<endl;
}

vector<State> Executor::process_new_plan(int sync_time_limit, Plan& plan_struct, vector<vector<Action>> & staged_actions)
{
    // Default implementation: always append, update the predicted states based on moves
    if (predicted_states.size() != env->num_of_agents)
    {
        predicted_states = env->system_states;
    }

    auto curr_states = predicted_states;

    if (env->system_timestep == 0)
    {
        //insert start locations to tpg
        for (int i = 0; i < env->num_of_agents; i++)
        {
            tpg[env->curr_states[i].location].push_back(i);
            previous_locations[i] = env->system_states[i].location;
        }
    }

    int moves[4] = {1, env->cols, -1, -env->cols};
    std::vector<std::vector<Action>> plan = plan_struct.actions;

    // //check how many actions we need to insert to ensure agents have enough actions to execute within the communication time limit.
    // int current_left_action_size = INT_MAX;
    // for (int i = 0; i < env->num_of_agents; i++)
    // {
    //     current_left_action_size = min(current_left_action_size, (int)staged_actions[i].size());
    // }
    // int num_insert_timesteps = window_size - current_left_action_size;

    // cout<<"current_left_action_size: "<<current_left_action_size<<endl;
    // cout<<"num_insert_timesteps: "<<num_insert_timesteps<<endl;

    //apppend actions to window size
    for (int timestep = 0; timestep < plan[0].size(); timestep++)
    {
        for (int i = 0; i < plan.size(); i++)
        {
            int new_location = curr_states[i].location;
            int new_orientation = curr_states[i].orientation;
            if (plan[i][timestep] == Action::FW)
            {
                new_location = new_location + moves[curr_states[i].orientation];
                tpg[new_location].push_back(i);
                // cout<<"insert "<<i<<" to tpg at "<<new_location<<endl;
            }
            else if (plan[i][timestep] == Action::CR)
            {
                new_orientation = (curr_states[i].orientation + 1) % 4;
            }
            else if (plan[i][timestep] == Action::CCR)
            {
                new_orientation = (curr_states[i].orientation - 1) % 4;
                if (new_orientation == -1)
                    new_orientation = 3;
            }
            // cout<<"agent "<<i<<" current prediceted state location from last iteration "<<predicted_states[i].location<<" orientation "<<predicted_states[i].orientation<<endl;
            // cout<<"agent "<<i<<" action "<<(plan[i][timestep] == Action::FW ? "FW" : (plan[i][timestep] == Action::CR ? "CR" : (plan[i][timestep] == Action::CCR ? "CCR" : (plan[i][timestep] == Action::NA ? "NA" : "W"))))<<" new prediceted state location "<<new_location<<" orientation "<<new_orientation<<endl;
            predicted_states[i].location = new_location;
            predicted_states[i].orientation = new_orientation;
            predicted_states[i].timestep+=1;
            curr_states[i] = predicted_states[i];
            // cout<<"agent "<<i<<" predicted state location "<<predicted_states[i].location<<" orientation "<<predicted_states[i].orientation<<" timestep "<<predicted_states[i].timestep<<endl;
            if (plan[i][timestep] != Action::NA && plan[i][timestep] != Action::W)
            {
                staged_actions[i].push_back(plan[i][timestep]);
            }
            //cout<<" t "<<timestep<<" agent "<<i<<" current location "<<curr_states[i].location<<" predict location "<<predicted_states[i].location<<endl;
        }
    }
    return predicted_states;
}

void Executor::next_command(int exec_time_limit, std::vector<ExecutionCommand> & agent_command)
{
    // cout<<"executor next_command with exec_time_limit: "<<exec_time_limit<<endl;
    // // //always go if there are staged actions
    // for (int i = 0; i < env->curr_states.size(); i++)
    // {
    //     if (!env->staged_actions[i].empty())
    //     {
    //         agent_command[i] = ExecutionCommand::GO;
    //     }
    //     else
    //     {
    //         agent_command[i] = ExecutionCommand::STOP;
    //     }
    // }
    // return;

    //update the tpg based on the current system states from last tick
    for (int i = 0; i < env->system_states.size(); i++)
    {
        int prev_location = previous_locations[i];
        int curr_location = env->system_states[i].location;
        // cout<<"tpg for agent "<<i<<" prev loc "<< prev_location<<" tpg first "<<tpg[prev_location].front()<<endl;
        assert(tpg[prev_location].front() == i);
        if (prev_location != curr_location && tpg[prev_location].front() == i)
        {
            //remove the agent from the previous location in tpg
            // cout<<"pop agent "<<i<<" from location "<< prev_location<<" current at "<<curr_location<<endl;
            tpg[prev_location].pop_front();
            previous_locations[i] = curr_location;
        }
    }
    vector<bool> decided(env->num_of_agents, false);
    //temp_tpg = tpg; //copy the tpg to temp_tpg for current timestep processing, we will update temp_tpg during mcp but keep tpg unchanged
    for (int i = 0; i < env->system_states.size(); i++)
    {
        // always try to go and clear orders because we don't know the current delay
        if (!decided[i])
            mcp(i, decided, agent_command);
    }
    
}

bool Executor::mcp(int agent_id, vector<bool> & curr_decision, std::vector<ExecutionCommand> & agent_command)
{
    // cout<<"mcp for "<<agent_id<<endl;
    if (env->staged_actions[agent_id].empty())
    {
        //no action, just stop and wait for the next plan, no tpg order clear
        agent_command[agent_id] = ExecutionCommand::STOP;
        curr_decision[agent_id] = true;
        return false;
    }
    else if (env->staged_actions[agent_id].front() != Action::FW) 
    {
        //try to go but still stay at current location because it's not move forward, no tpg order clear
        agent_command[agent_id] = ExecutionCommand::GO;
        curr_decision[agent_id] = true;
        return false;
    }
    else
    {
        int moves[4] = {1, env->cols, -1, -env->cols};
        int curr_location = env->system_states[agent_id].location;
        int curr_orientation = env->system_states[agent_id].orientation;
        int next_location = curr_location + moves[curr_orientation];

        if (tpg[next_location].empty() || tpg[next_location].front() == agent_id)
        {
            //try to go and can go, clear the tpg order
            agent_command[agent_id] = ExecutionCommand::GO;
            // if (!tpg[curr_location].empty())
            //     tpg[curr_location].pop_front();

            curr_decision[agent_id] = true;
            // cout<<"mcp true, agent "<<agent_id<<" decided to move to "<<next_location<<endl;
            return true;
        }
        else
        {
            // //try to go by recursion
            // int blocking_agent_id = temp_tpg[next_location].front();

            // cout<<"agent "<<agent_id<<" next loc "<<next_location<<" tpg blocking agent "<<blocking_agent_id<<endl;

            // int next_id=-1;
            // auto it = temp_tpg[next_location].begin();
            // if (it != temp_tpg[next_location].end()) 
            // {
            //     auto it2 = std::next(it);
            //     if (it2 != temp_tpg[next_location].end()) 
            //     {
            //         // *it2 is the second element
            //         next_id = *it2;
            //     }
            // }

            // assert(next_id != -1); 
            
            // if (curr_decision[blocking_agent_id] || next_id != agent_id)
            // {                
            //     //the blocking agent has already made the decision or current agent is not on the second order, so we cannot go
            //     agent_command[agent_id] = ExecutionCommand::STOP;
            //     curr_decision[agent_id] = true;
            //     return false;       
            // }

            // //try to simulate to see if the agent can go by pushing blocking agent
            // curr_decision[agent_id] = true;
            // if (!temp_tpg[curr_location].empty())
            //     temp_tpg[curr_location].pop_front(); //temporarily pop the current agent from temp_tpg to simulate the move

            // if (mcp(blocking_agent_id, curr_decision, agent_command) && temp_tpg[next_location].front() == agent_id)
            // {

            //     //now agent can go and clear the tpg order
            //     agent_command[agent_id] = ExecutionCommand::GO;
            //     cout<<"mcp true, agent "<<agent_id<<" decided to move to "<<next_location<<endl;
            //     // curr_decision[agent_id] = true;
            //     return true;
            // }
            // else
            // {
                //the blocking agent cannot go, so we cannot go
                agent_command[agent_id] = ExecutionCommand::STOP;
                curr_decision[agent_id] = true;
                return false;
            // }
        }
    }
    
}

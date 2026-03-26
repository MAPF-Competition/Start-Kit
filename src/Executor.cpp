#include "Executor.h"

void Executor::initialize(int preprocess_time_limit)
{
    window_size = env->min_planner_communication_time / (env->action_time*env->max_counter); //calculate the window size based on the communication time limit and action execution time
    if (window_size < 1)
        window_size = 1;
    previous_locations.resize(env->num_of_agents, -1);
}

vector<State> Executor::process_new_plan(int sync_time_limit, Plan& plan_struct, vector<vector<Action>> & staged_actions)
{
    // Default implementation: always append, update the predicted states based on moves
    if (predicted_states.size() != env->num_of_agents)
    {
        predicted_states = env->system_states;
    }

    for (int i = 0; i < env->num_of_agents; i++)
    {
        previous_locations[i] = env->system_states[i].location; //for use of keep track of tpg in move function
        if (env->system_timestep == 0)//insert start location to tpg
        {
            tpg[env->curr_states[i].location].push_back(i);
        }
    }

    auto curr_states = predicted_states;

    int moves[4] = {1, env->cols, -1, -env->cols};
    std::vector<std::vector<Action>> plan = plan_struct.actions;

    //first apppend all actions and tpg location to window size
    for (int timestep = 0; timestep < plan[0].size() && timestep < window_size; timestep++)
    {
        for (int i = 0; i < plan.size(); i++)
        {
            int new_location = curr_states[i].location;
            int new_orientation = curr_states[i].orientation;

            if (plan[i][timestep] == Action::FW)
            {
                new_location = new_location + moves[curr_states[i].orientation];
                tpg[new_location].push_back(i);
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
            curr_states[i].location = new_location;
            curr_states[i].orientation = new_orientation;   
            curr_states[i].timestep+=1;

            if (plan[i][timestep] != Action::NA && plan[i][timestep] != Action::W)
            {
                staged_actions[i].push_back(plan[i][timestep]);
            }
        }
    }

    //now run a mcp simulation assuming no delays for a windowed size, and obtain the actions
    vector<int> action_index(env->num_of_agents,0); //keep track of the next action index for each agent in the staged actions 
    curr_states = env->system_states;
    temp_tpg.clear();

    for (int t = 0; t < window_size; t++)
    {
        auto pre_states = curr_states;
        for (int i = 0; i < env->num_of_agents; i++)
        {

            if (staged_actions[i].size() <= action_index[i])//no action left to proceed
            {
                curr_states[i].timestep++;//treat as wait
                continue;
            }

            //simulate with mcp (no delay)
            Action curr_action = staged_actions[i][action_index[i]];
            int curr_location = curr_states[i].location;
            int curr_orientation = curr_states[i].orientation;
            int new_location = curr_location;
            int new_orientation = curr_orientation;

            assert(tpg[curr_location].front() == i);

            if (curr_action == Action::FW)
            {
                new_location = curr_location + moves[curr_orientation];
                assert(!tpg[new_location].empty());
                if (tpg[new_location].front() == i)
                {
                    //you can go
                    action_index[i]++;
                }
                else
                {
                    new_location = curr_location; //cannot move because of the dependency, stay at the current location
                }
            }
            else if (curr_action == Action::CR)
            {
                new_orientation = (curr_orientation + 1) % 4;
                //you can go because it's just rotation
                action_index[i]++;
            }
            else if (curr_action == Action::CCR)
            {
                new_orientation = (curr_orientation - 1) % 4;
                if (new_orientation == -1)
                    new_orientation = 3;
                //you can go because it's just rotation
                action_index[i]++;
            }
            curr_states[i].location = new_location;
            curr_states[i].orientation = new_orientation;   
        }
        //clear dependency
        for (int i = 0; i < env->num_of_agents; i++)
        {
            if (t == 0)
            {
                temp_tpg[pre_states[i].location].push_back(i);
                // cout<<"inserting "<<i<<" into tpg loc "<<pre_states[i].location<<endl;
            }

            if (pre_states[i].location != curr_states[i].location) //agent move
            {
                //insert location into temp tpg, and pop the current location from tpg
                assert(tpg[pre_states[i].location].front() == i);
                tpg[pre_states[i].location].pop_front();
                temp_tpg[curr_states[i].location].push_back(i);
                // cout<<"inserting "<<i<<" into tpg loc "<<curr_states[i].location<<endl;
            }
        }
    }
    for (int i = 0; i < env->num_of_agents; i++)
    {
        if (action_index[i] < staged_actions[i].size())
        {
            staged_actions[i].resize(action_index[i]);//remove the actions that are not executed in the windowed simulation
            // cout<<"resizing staged actions for agent "<<i<<" to "<<action_index[i]<<endl;
        }
    }
    predicted_states = curr_states;
    tpg = temp_tpg;
    return predicted_states;
}

void Executor::next_command(int exec_time_limit, std::vector<ExecutionCommand> & agent_command)
{
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
    for (int i = 0; i < env->system_states.size(); i++)
    {
        mcp(i, agent_command);
    }
    
}

bool Executor::mcp(int agent_id, std::vector<ExecutionCommand> & agent_command)
{
    // cout<<"mcp for "<<agent_id<<endl;
    if (env->staged_actions[agent_id].empty())
    {
        //no action, just stop and wait for the next plan, no tpg order clear
        agent_command[agent_id] = ExecutionCommand::STOP;
        return false;
    }
    else if (env->staged_actions[agent_id].front() != Action::FW) 
    {
        //try to go but still stay at current location because it's not move forward, no tpg order clear
        agent_command[agent_id] = ExecutionCommand::GO;
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
                return false;
            // }
        }
    }
    
}

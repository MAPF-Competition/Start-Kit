#include "Simulator.h"
#include "nlohmann/json.hpp"
using json = nlohmann::ordered_json;

bool Simulator::initialise_executor(int preprocess_time_limit)
{
    if (executor == nullptr)
    {
        return false;
    }
    executor->initialize(preprocess_time_limit);
    return true;
}

void Simulator::process_new_plan(int sync_time_limit, int overtime_runtime, Plan& plan) 
{
    //call executor to process the new plan and get staged actions
    auto process_start = std::chrono::steady_clock::now();
    //todo: change plan to vector<vector<Action>>
    predict_states =  executor->process_new_plan(sync_time_limit, plan, staged_actions);
    auto process_end = std::chrono::steady_clock::now();
    int diff = (int)std::chrono::duration_cast<std::chrono::milliseconds>(process_end - process_start).count() - overtime_runtime;
    //timeout execute all wait
    while (diff > 0)
    {
        auto dummy_actions = std::vector<Action>(num_of_agents, Action::W);
        move(overtime_runtime);
        diff -= overtime_runtime;
    }   
}

vector<State> Simulator::move(int move_time_limit) //move one single 100ms step 
{
    cout<<"timestep "<<timestep<<endl;
    //first call executor to get next execution command for each agent based on current state and staged actions
    std::vector<ExecutionCommand> agent_command;
    // reserve space for the executor to write commands
    agent_command.resize(num_of_agents);

    auto process_start = std::chrono::steady_clock::now();
    executor->next_command(move_time_limit, agent_command);
    auto process_end = std::chrono::steady_clock::now();
    int diff = (int)std::chrono::duration_cast<std::chrono::milliseconds>(process_end - process_start).count() - move_time_limit;
    std::vector<Action> actions(num_of_agents, Action::W); //default action is wait

    simulate_delay();

    while (diff > 0)
    {
        cout<<"time out wait"<<endl;
        for (int i = 0; i < num_of_agents; i++)
        {
            planner_movements[i].push_back(Action::NA);
            actual_movements[i].push_back(Action::W);
        }
        timestep++; //all agents wait for one timestep
        diff -= move_time_limit;
        simulate_delay();
    } 

    //process the actions based on the execution command
    for (int i = 0; i < num_of_agents; i++)
    {
        if (agent_command[i] == ExecutionCommand::GO)
        {
            if (staged_actions[i].empty())
            {
                actions[i] = Action::W;
            }
            else
            {
                actions[i] = staged_actions[i].front();
            }
        }
        else //STOP
        {
            actions[i] = Action::W;
        }
        if (i == 22 || i == 84)
        {
            cout<<"agent "<<i<<" command "<<(agent_command[i] == ExecutionCommand::GO ? "GO" : "STOP")<<" action "<<(actions[i] == Action::FW ? "FW" : (actions[i] == Action::CR ? "CR" : (actions[i] == Action::CCR ? "CCR" : (actions[i] == Action::NA ? "NA" : "W"))))<<endl;
            cout<<"current states"<<curr_states[i].location<<" orientation "<<curr_states[i].orientation<<" counter "<<curr_states[i].counter.count<<endl;
        }
        if (curr_states[i].delay.inDelay)
        {
            actions[i] = Action::W;
        }
        planner_movements[i].push_back(actions[i]);
    }

    auto pre_states = curr_states;

    curr_states = model->step(curr_states, actions,timestep);
    timestep++;

    //clear staged actions if the action is executed (either wait or the actual action) and update the staged actions for the next tick
    for (int k = 0; k < num_of_agents; k++)
    {
        //record the actual path and actions
        paths[k].push_back(curr_states[k]);
        actual_movements[k].push_back(actions[k]);

        if (staged_actions[k].empty())
        {
            continue;
        }
        //staged action is wait and the agent actually wait
        if (staged_actions[k].front() == Action::W && actions[k] == Action::W) //staged action is wait and the agent actually wait
        {
            staged_actions[k].erase(staged_actions[k].begin());
        }
        //  //staged action is the same as actual action and the agent has finished the move (counter is 0 after the tick in result_state)
        // else if (actions[k] != Action::W && staged_actions[k].front() == actions[k] && curr_states[k].counter.count == 0)
        // {
        //     staged_actions[k].erase(staged_actions[k].begin());
        // }
        else if (pre_states[k].location != curr_states[k].location || pre_states[k].orientation != curr_states[k].orientation)
        {
            //the agent has moved to the next location or move to next orientation, so we can remove the staged action
            staged_actions[k].erase(staged_actions[k].begin());
        }
    }
    //return move_valid;
    return curr_states;
}

void Simulator::record_planned_movements(Action action, int agent_id)
{
    
    
}

void Simulator::simulate_delay()
{
    std::vector<bool> started_this_tick(num_of_agents, false);

    if (timestep >= 0 && timestep < static_cast<int>(delay_schedule.size()))
    {
        for (const auto& scheduled_delay : delay_schedule[timestep])
        {
            int agent = scheduled_delay.first;
            int duration = scheduled_delay.second;
            if (agent < 0 || agent >= num_of_agents || duration <= 0)
            {
                continue;
            }

            if (!curr_states[agent].delay.inDelay)
            {
                curr_states[agent].delay.inDelay = true;
                delays[agent] = duration;
                started_this_tick[agent] = true;
                cout<<"agent "<<agent<<" starts delay for "<<delays[agent]<<" timesteps"<<endl;
            }
            else
            {
                delays[agent] = std::max(delays[agent], duration);
            }
        }
    }

    for (int k = 0; k < num_of_agents; k++)
    {
        if (curr_states[k].delay.inDelay && !started_this_tick[k])
        {
            delays[k]--;
            if (delays[k] <= 0)
            {
                curr_states[k].delay.inDelay = false;
            }
        }
    }
}

void Simulator::validate_actions_with_delay(vector<Action>& actions) 
{
    for (int k = 0; k < num_of_agents; k++)
    {
        if (curr_states[k].delay.inDelay)
        {
            //if the agent is in delay, it can only wait. 
            actions[k] = Action::W;
        }
    }
}

void Simulator::sync_shared_env(SharedEnvironment* env) 
{
    // update the shared environment with simulator's current state
    env->curr_states = predict_states;
    env->system_states = curr_states;
    env->start_states = predict_states;
    // env->curr_states = curr_states;
    env->system_timestep = timestep;

    env->staged_actions = staged_actions;
    

    // make sure executor uses the same shared environment
    if (executor != nullptr)
    {
        executor->env = env;
    }
}

json Simulator::actual_path_to_json() const
{
    // Save actual paths
    json apaths = json::array();
    for (int i = 0; i < num_of_agents; i++)
    {
        std::string path;
        bool first = true;
        for (const auto action : actual_movements[i])
        {
            if (!first)
            {
                path+= ",";
            }
            else
            {
                first = false;
            }

            if (action == Action::FW)
            {
                path+="F";
            }
            else if (action == Action::CR)
            {
                path+="R";
            } 
            else if (action == Action::CCR)
            {
                path+="C";
            }
            else if (action == Action::NA)
            {
                path+="T";
            }
            else
            {
                path+="W";
            }
        }
        apaths.push_back(path);
    }

    return apaths;
}

json Simulator::planned_path_to_json() const
{
    //planned paths
    json ppaths = json::array();
    for (int i = 0; i < num_of_agents; i++)
    {
        std::string path;
        bool first = true;
        for (const auto action : planner_movements[i])
        {
            if (!first)
            {
                path+= ",";
            } 
            else 
            {
                first = false;
            }

            if (action == Action::FW)
            {
                path+="F";
            }
            else if (action == Action::CR)
            {
                path+="R";
            } 
            else if (action == Action::CCR)
            {
                path+="C";
            } 
            else if (action == Action::NA)
            {
                path+="T";
            }
            else
            {
                path+="W";
            }
        }  
        ppaths.push_back(path);
    }

    return ppaths;
}

json Simulator::starts_to_json() const
{
    json start = json::array();
    for (int i = 0; i < num_of_agents; i++)
    {
        json s = json::array();
        s.push_back(starts[i].location/map.cols);
        s.push_back(starts[i].location%map.cols);
        switch (starts[i].orientation)
        {
        case 0:
            s.push_back("E");
            break;
        case 1:
            s.push_back("S");
        case 2:
            s.push_back("W");
            break;
        case 3:
            s.push_back("N");
            break;
        }
        start.push_back(s);
    }

    return start;
}

json Simulator::action_errors_to_json() const
{
    // Save errors
    json errors = json::array();
    for (auto error: model->errors)
    {
        std::string error_msg;
        int agent1;
        int agent2;
        int timestep;
        std::tie(error_msg,agent1,agent2,timestep) = error;
        json e = json::array();
        e.push_back(agent1);
        e.push_back(agent2);
        e.push_back(timestep);
        e.push_back(error_msg);
        errors.push_back(e);
    }

    return errors;
}

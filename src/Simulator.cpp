#include "Simulator.h"
#include "nlohmann/json.hpp"
using json = nlohmann::ordered_json;

vector<State> Simulator::process_new_plan(int sync_time_limit, vector<Action>& plan) 
{
    //call executor to process the new plan and get staged actions
    std::cout<<"call"<<std::endl;
    auto states =  executor->process_new_plan(sync_time_limit, plan, staged_actions);
    std::cout<<"returned"<<std::endl;
    return states;
}

vector<State> Simulator::move(vector<Action>& actions) //move one single 100ms step 
{
    //first call executor to get next execution command for each agent based on current state and staged actions
    std::vector<ExecutionCommand> agent_command;
    // reserve space for the executor to write commands
    agent_command.resize(num_of_agents);
    executor->next_command(100, curr_states, staged_actions, agent_command);

    for (int k = 0; k < num_of_agents; k++)
    {    
        if (k >= actions.size()){
            planner_movements[k].push_back(Action::NA);
        }
        else
        {
            if (curr_states[k].delay.inDelay() && actions[k] != Action::W){
                //if the agent is in delay, it can only wait. 
                planner_movements[k].push_back(Action::W);
                //FIXME: Should we do the tick here? This depends on where we want to call the executor.
                curr_states[k].delay.tick();
            }
            else
            {
                planner_movements[k].push_back(actions[k]);
            }
        }
    }

    //execute the actions based on the execution command
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
    }

    //validate the actions with delays
    validate_actions_with_delay(actions);

    //validate the action with agent models
    if (!model->is_valid(curr_states, actions,timestep))
    {
        //move_valid = false;
        all_valid = false;
        actions = std::vector<Action>(num_of_agents, Action::W);
    }

    curr_states = model->result_states(curr_states, actions);
    timestep++;

    for (int k = 0; k < num_of_agents; k++){
        paths[k].push_back(curr_states[k]);
        actual_movements[k].push_back(actions[k]);
    }
    //return move_valid;
    return curr_states;
}

void Simulator::validate_actions_with_delay(vector<Action>& actions) 
{
    //currently no delay model
    return;
}

void Simulator::sync_shared_env(SharedEnvironment* env) 
{
    // update the shared environment with simulator's current state
    env->curr_states = curr_states;
    env->curr_timestep = timestep;

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

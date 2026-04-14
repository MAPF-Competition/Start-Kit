#include "Simulator.h"
#include "nlohmann/json.hpp"
#include <sstream>
#include <stdexcept>
using json = nlohmann::ordered_json;

namespace {

std::vector<Action> normalize_plan_actions(const std::vector<Action>& actions)
{
    std::vector<Action> normalized;
    normalized.reserve(actions.size());
    for (Action action : actions)
    {
        if (action != Action::W && action != Action::NA)
        {
            normalized.push_back(action);
        }
    }
    return normalized;
}

std::string actions_to_string(const std::vector<Action>& actions)
{
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < actions.size(); i++)
    {
        if (i > 0)
        {
            oss << ", ";
        }
        switch (actions[i])
        {
            case Action::FW: oss << "FW"; break;
            case Action::CR: oss << "CR"; break;
            case Action::CCR: oss << "CCR"; break;
            case Action::W: oss << "W"; break;
            case Action::NA: oss << "NA"; break;
            default: oss << "?"; break;
        }
    }
    oss << "]";
    return oss.str();
}

void validate_staged_actions_prefix(
    const std::vector<std::vector<Action>>& previous_staged_actions,
    const std::vector<std::vector<Action>>& planned_actions,
    const std::vector<std::vector<Action>>& returned_staged_actions)
{
    if (planned_actions.size() != previous_staged_actions.size() ||
        returned_staged_actions.size() != previous_staged_actions.size())
    {
        throw std::runtime_error("Executor returned staged actions with unexpected agent count");
    }

    for (size_t agent = 0; agent < previous_staged_actions.size(); agent++)
    {
        std::vector<Action> expected = normalize_plan_actions(previous_staged_actions[agent]);
        std::vector<Action> normalized = normalize_plan_actions(planned_actions[agent]);
        expected.insert(expected.end(), normalized.begin(), normalized.end());

        std::vector<Action> actual = normalize_plan_actions(returned_staged_actions[agent]);
        if (actual.size() > expected.size())
        {
            std::ostringstream oss;
            oss << "Executor returned too many staged actions for agent " << agent
                << ": expected a prefix of " << actions_to_string(expected)
                << ", got " << actions_to_string(actual);
            throw std::runtime_error(oss.str());
        }

        for (size_t idx = 0; idx < actual.size(); idx++)
        {
            if (actual[idx] != expected[idx])
            {
                std::ostringstream oss;
                oss << "Executor returned invalid staged actions for agent " << agent
                    << ": expected a prefix of " << actions_to_string(expected)
                    << ", got " << actions_to_string(actual);
                throw std::runtime_error(oss.str());
            }
        }
    }
}

} // namespace

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
    const std::vector<std::vector<Action>> previous_staged_actions = staged_actions;
    const std::vector<std::vector<Action>> planned_actions = plan.convert_to_actions();

    //call executor to process the new plan and get staged actions
    auto process_start = std::chrono::steady_clock::now();
    predict_states = executor->process_new_plan(sync_time_limit, plan, staged_actions);
    auto process_end = std::chrono::steady_clock::now();

    if (executor_validation)
        validate_staged_actions_prefix(previous_staged_actions, planned_actions, staged_actions);

    int diff = (int)std::chrono::duration_cast<std::chrono::milliseconds>(process_end - process_start).count() - overtime_runtime;
    //timeout execute all wait
    while (diff > 0)
    {
        move_all_wait(1); //all agents wait for one timestep
        diff -= overtime_runtime;
    }   
}

void Simulator::move_all_wait(int steps) //move only wait action for all agents, used for timeout movement when processing new plan
{
    timestep+=steps;
}

vector<State> Simulator::move(int move_time_limit) //move one single 100ms step 
{
    // cout<<"timestep "<<timestep<<endl;
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
        // cout<<"time out wait"<<endl;
        for (int i = 0; i < num_of_agents; i++)
        {
            record_planned_movements(Action::NA, i);
            record_actual_movements(curr_states[i], Action::W, i);
        }
        move_all_wait(1); //all agents wait for one timestep
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
        if (curr_states[i].delay.inDelay)
        {
            // cout<<"agent "<<i<<" delayed"<<endl;
            actions[i] = Action::W;
        }
        record_planned_movements(actions[i], i);
        // if (actions[i] == Action::FW)

            // cout<<"planned movement for agent "<<i<<" action fw"<<endl;
    }

    auto pre_states = curr_states;

    curr_states = model->step(curr_states, actions,timestep);
    timestep++;

    //clear staged actions if the action is executed (either wait or the actual action) and update the staged actions for the next tick
    for (int k = 0; k < num_of_agents; k++)
    {
        //record the actual path and actions
        record_actual_movements(pre_states[k], actions[k], k);

        // if (actions[k] == Action::FW)

        //     cout<<"actual movement for agent "<<k<<" action fw"<<endl;

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
        // cout<<"executor, current agent "<<k<<" pre state "<<pre_states[k].location<<" curr state "<<curr_states[k].location<<endl;
    }
    //return move_valid;
    return curr_states;
}

void Simulator::record_planned_movements(Action action, int agent_id)
{
    assert(current_planner_chunk_index[agent_id] < chunked_planner_movements[agent_id].size());
    bool is_different_action = true;
    if (current_planner_chunk_count[agent_id] > 0)
    {
        assert(!chunked_planner_movements[agent_id][current_planner_chunk_index[agent_id]].empty());
        auto last_movement = chunked_planner_movements[agent_id][current_planner_chunk_index[agent_id]].back().first;
        if (last_movement == action)
        {
            is_different_action = false;
        }
    }
    if (is_different_action)
    {
        chunked_planner_movements[agent_id][current_planner_chunk_index[agent_id]].push_back({action, 1});
    }
    else
    {
        //update the last movement's timestep to current timestep
        chunked_planner_movements[agent_id][current_planner_chunk_index[agent_id]].back().second++;
    }
    if (current_planner_chunk_count[agent_id] == 0)
    {
        chunked_planner_snapshot_states[agent_id][current_planner_chunk_index[agent_id]] = curr_states[agent_id];
    }
    current_planner_chunk_count[agent_id]++;
    if (current_planner_chunk_count[agent_id] >= chunk_size)
    {
        current_planner_chunk_index[agent_id]++;
        current_planner_chunk_count[agent_id] = 0;
    }
}

void Simulator::record_actual_movements(State state, Action action, int agent_id)
{
    assert(current_actual_chunk_index[agent_id] < chunked_actual_movements[agent_id].size());
    bool is_different_action = true;
    if (current_actual_chunk_count[agent_id] > 0)
    {
        assert(!chunked_actual_movements[agent_id][current_actual_chunk_index[agent_id]].empty());
        auto last_movement = chunked_actual_movements[agent_id][current_actual_chunk_index[agent_id]].back().first;
        if (last_movement == action)
        {
            is_different_action = false;
        }
    }
    if (is_different_action)
    {
        chunked_actual_movements[agent_id][current_actual_chunk_index[agent_id]].push_back({action, 1});
    }
    else
    {
        //update the last movement's timestep to current timestep
        chunked_actual_movements[agent_id][current_actual_chunk_index[agent_id]].back().second++;
    }
    if (current_actual_chunk_count[agent_id] == 0)
    {
        chunked_actual_snapshot_states[agent_id][current_actual_chunk_index[agent_id]] = state;
    }
    current_actual_chunk_count[agent_id]++;
    if (current_actual_chunk_count[agent_id] >= chunk_size)
    {
        current_actual_chunk_index[agent_id]++;
        current_actual_chunk_count[agent_id] = 0;
    }
}


void Simulator::simulate_delay()
{
    if (!delay_enabled || delay_generator == nullptr)
    {
        return;
    }

    delay_generator->nextTick();
    const auto& remaining_delays = delay_generator->get_remaining_delays();
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        curr_states[agent].delay.inDelay = remaining_delays[agent] > 0;
    }
}

void Simulator::set_delay_enabled(bool enabled)
{
    delay_enabled = enabled;
    if (delay_enabled)
    {
        return;
    }

    for (int i = 0; i < num_of_agents; i++)
    {
        curr_states[i].delay.inDelay = false;
        predict_states[i].delay.inDelay = false;
    }
    if (delay_generator != nullptr)
    {
        delay_generator->clear_active_delays();
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
}

json Simulator::actual_path_to_json() const
{
    // Save actual paths
    json apaths = json::array();
    for (int i = 0; i < num_of_agents; i++)
    {
        std::string path;
        int curr_steps = 0;
        for (const auto& chunk : chunked_actual_movements[i])
        {
            path+="[(";
            path+=std::to_string(curr_steps);
            path+=",";
            path+=std::to_string(chunked_actual_snapshot_states[i][0].location/map.cols);
            path+=","; 
            path+=std::to_string(chunked_actual_snapshot_states[i][0].location%map.cols);
            path+=",";
            path+=std::to_string(chunked_actual_snapshot_states[i][0].orientation);
            path+= ",";
            path+=std::to_string(chunked_actual_snapshot_states[i][0].counter.count);
            path+="):(";
            
            bool first = true;
            for (const auto& action_pair : chunk)
            {
                Action action = action_pair.first;
                int duration = action_pair.second;
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
                path+=" ";
                path+=std::to_string(duration);
            }
            path+=")]";
            curr_steps += chunk_size;
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
        int curr_steps = 0;
        for (const auto& chunk : chunked_planner_movements[i])
        {
            path+="[(";
            path+=std::to_string(curr_steps);
            path+=",";
            path+=std::to_string(chunked_planner_snapshot_states[i][0].location/map.cols);
            path+=","; 
            path+=std::to_string(chunked_planner_snapshot_states[i][0].location%map.cols);
            path+=",";
            path+=std::to_string(chunked_planner_snapshot_states[i][0].orientation);
            path+= ",";
            path+=std::to_string(chunked_planner_snapshot_states[i][0].counter.count);
            path+="):(";
            bool first = true;
            for (const auto& action_pair : chunk)
            {
                Action action = action_pair.first;
                int duration = action_pair.second;
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
                path+=" ";
                path+=std::to_string(duration);
            }
            curr_steps += chunk_size;
            path+=")]";
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

#include "Executor.h"

void Executor::initialize(int preprocess_time_limit)
{
    // Default implementation does nothing
}
vector<State> Executor::process_new_plan(int sync_time_limit, vector<Action> plan, vector<vector<Action>> & staged_actions)
{
    // Default implementation: always append, update the predicted states based on moves
    vector<State> curr_states = env->curr_states;
    vector<State> predicted_states(env->num_of_agents);
    int moves[4] = {1, env->cols, -1, -env->cols};
    for (int i = 0; i < plan.size(); i++)
    {
        int new_location = curr_states[i].location;
        int new_orientation = curr_states[i].orientation;
        if (plan[i] == Action::FW)
        {
            new_location = new_location += moves[curr_states[i].orientation];
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
        // else
        // {
        //     for (int count = 0; count < 10; count++)
        //     {
        //         staged_actions[i].push_back(Action::W);
        //     }
        // }
    }
    return predicted_states;
}

void Executor::next_command(int exec_time_limit, std::vector<vector<Action>> staged_actions, std::vector<ExecutionCommand> & agent_command)
{
    // Default implementation: always GO if there are staged actions
    for (int i = 0; i < env->curr_states.size(); i++)
    {
        if (staged_actions[i].empty())
        {
            agent_command[i] = ExecutionCommand::STOP;
        }
        else
        {
            agent_command[i] = ExecutionCommand::GO;
        }
    }
}


#include "Executor.h"

#include "default_executor.h"

void Executor::initialize(int preprocess_time_limit)
{
    // window_size = (int)env->min_planner_communication_time / (env->action_time*env->max_counter) + 1; //calculate the window size based on the communication time limit and action execution time
    // previous_locations.resize(env->num_of_agents, -1);
    DefaultPlanner::execute_initialize(preprocess_time_limit, env);
}

vector<State> Executor::process_new_plan(int sync_time_limit, Plan& plan_struct, vector<vector<Action>> & staged_actions)
{
    return DefaultPlanner::execute_process_new_plan(sync_time_limit, plan_struct, staged_actions, env);
}

void Executor::next_command(int exec_time_limit, std::vector<ExecutionCommand> & agent_command)
{
    DefaultPlanner::execute_next_command(exec_time_limit, agent_command, env);
    
}

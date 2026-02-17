#pragma once
#include "ActionModel.h"
#include "SharedEnv.h"

class Executor
{
public:
    SharedEnvironment* env;
    Executor(SharedEnvironment* env): env(env){};
    Executor(){env = new SharedEnvironment();};
    virtual ~Executor(){delete env;};   
    
    // initialisation before processing any plan
    virtual void initialize(int preprocess_time_limit);
    // return the predicted states after processing the new plan, and staged actions for each agent
    virtual vector<State> process_new_plan(int sync_time_limit, vector<Action> plan, vector<vector<Action>> & staged_actions);
    // return the next execution command for each agent based on the current state and staged actions
    virtual void next_command(int exec_time_limit, std::vector<vector<Action>> staged_actions, std::vector<ExecutionCommand> & agent_command);

    vector<list<int>> tpg; //dependency graph for location visiting orders
    vector<list<int>> temp_tpg; //temporary dependency graph for current timestep, used for mcp
    vector<int> previous_locations; //record the previous locations of agents for tpg update

    bool mcp(std::vector<vector<Action>> staged_actions, int agent_id, vector<bool> & curr_decision, std::vector<ExecutionCommand> & agent_command);

};
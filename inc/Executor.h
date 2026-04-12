#pragma once
#include "ActionModel.h"
#include "SharedEnv.h"
#include "Plan.h"

enum ProcessPlanType {APPEND_ONE, APPEND_ALL, APPEND_WINDOW};

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
    virtual vector<State> process_new_plan(int sync_time_limit, Plan & plan, vector<vector<Action>> & staged_actions);
    // return the next execution command for each agent based on the current state and staged actions
    virtual void next_command(int exec_time_limit, std::vector<ExecutionCommand> & agent_command);

    vector<list<int>> tpg; //dependency graph for location visiting orders
    //vector<list<int>> temp_tpg; //temporary dependency graph for current timestep, used for mcp
    vector<int> previous_locations; //record the previous locations of agents for tpg update
    vector<State> predicted_states;

    ProcessPlanType process_plan_type = APPEND_WINDOW; //the default process plan type is to append windoed actions to staged actions, window = min_planner_communication_time/simulator_time_limit, which means we will only execute the part of the plan that can be executed within the communication time limit.
    int window_size;

    bool mcp(int agent_id, vector<bool> & curr_decision, std::vector<ExecutionCommand> & agent_command);

};
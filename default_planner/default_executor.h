

#include "Types.h"

namespace DefaultPlanner{

void execute_initialize(int preprocess_time_limit, SharedEnvironment* env);

vector<State> execute_process_new_plan(int sync_time_limit, Plan & plan, vector<vector<Action>> & staged_actions, SharedEnvironment* env);

void execute_next_command(int exec_time_limit, std::vector<ExecutionCommand> & agent_command, SharedEnvironment* env);

bool mcp(int agent_id, std::vector<ExecutionCommand> & agent_command, SharedEnvironment* env);

}
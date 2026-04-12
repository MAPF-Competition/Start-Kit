


// Default executor baseline used by src/Executor.cpp.
//
// Responsibilities:
// 1) process_new_plan: stage planner actions into executable windowed actions and
//    return predicted states for the next planning episode.
// 2) next_command: output per-agent GO/STOP each tick under execution uncertainty.
//
// GO/STOP decisions are derived from a Temporal Dependency Graph (TPG) that
// captures the implied visiting order of each location in the planned paths.
// When an agent is delayed, its dependent successors are automatically held.
//
// Reference:
//   Ma, H., Kumar, T. S., & Koenig, S. (2017). Multi-agent path finding with
//   delay probabilities. AAAI Conference on Artificial Intelligence, Vol. 31.
#include "Types.h"

namespace DefaultPlanner{

void execute_initialize(int preprocess_time_limit, SharedEnvironment* env);

vector<State> execute_process_new_plan(int sync_time_limit, Plan & plan, vector<vector<Action>> & staged_actions, SharedEnvironment* env);

void execute_next_command(int exec_time_limit, std::vector<ExecutionCommand> & agent_command, SharedEnvironment* env);

bool mcp(int agent_id, std::vector<ExecutionCommand> & agent_command, SharedEnvironment* env);

}
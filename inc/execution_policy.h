#include "SharedEnv.h"
#include "planner_wrapper.h"

template <class P> class ExecutionPolicy {

public:
  ExecutionPolicy(P *planner) : planner_(planner_) {}

  std::vector<Action> &get_actions(const SharedEnvironment &env) {
    return planner_->query(env.curr_states, env.goal_locations);
  }

private:
  const P *planner_;
};

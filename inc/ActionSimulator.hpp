#include "ActionModel.h"
#include "SharedEnv.h"
#include "Status.hpp"
#include <random>

class ActionSimulator {
public:
  ActionSimulator(ActionModelWithRotate &model, SharedEnvironment *env)
      : model(model), env(env){};
  virtual vector<Status> simulate_action(vector<Action> &next_actions);
  virtual bool validate_safe();

protected:
  ActionModelWithRotate &model;
  SharedEnvironment *env;
};

// Classical MAPF scenario where all actions succeed
class PerfectSimulator : ActionSimulator {
public:
  PerfectSimulator(ActionModelWithRotate &model, SharedEnvironment *env)
      : ActionSimulator(model, env){};

  vector<Status> simulate_action(SharedEnvironment *env,
                                 vector<Action> &next_actions) {
    env->curr_states = model.result_states(env->curr_states, next_actions);
    return vector<Status>(env->num_of_agents, Status::SUCCESS);
  }
};

// Implements delay probability for MAPF-DP
class ProbabilisticSimulator : ActionSimulator {
public:
  ProbabilisticSimulator(float success_chance, ActionModelWithRotate &model,
                         SharedEnvironment *env)
      : success_chance_(success_chance), rd_(), gen_(rd_()), distrib_(0, 1),
        ActionSimulator(model, env){};

  vector<Status> simulate_action(vector<Action> &next_actions) override {
    vector<Status> progress(env->num_of_agents);
    for (int i = 0; i < env->num_of_agents; i++) {
      // Succeeds success_chance % of the time, never if <=0 and always if >=1
      if (success_chance_ > distrib_(gen_)) {
        progress[i] = Status::SUCCESS;
        State &curr = env->curr_states.at(i);
        curr = model.result_state(curr, next_actions.at(i));
      } else {
        progress[i] = Status::FAILED;
      }
    }
    return progress;
  }

private:
  float success_chance_;
  std::random_device rd_;
  std::mt19937 gen_;
  std::uniform_real_distribution<double> distrib_;
};

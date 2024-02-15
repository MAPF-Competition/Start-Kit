#include "ActionModel.h"
#include "SharedEnv.h"
#include "Status.hpp"
#include <random>

class ActionSimulator {
public:
  ActionSimulator(ActionModelWithRotate &model) : model_(model){};
  virtual vector<Status> simulate_action(SharedEnvironment *env,
                                         vector<Action> &next_actions);

protected:
  ActionModelWithRotate model_;
};

class PerfectSimulator : ActionSimulator {
public:
  PerfectSimulator(ActionModelWithRotate model) : ActionSimulator(model){};

  vector<Status> simulate_action(SharedEnvironment *env,
                                 vector<Action> &next_actions) {
    env->curr_states = model_.result_states(env->curr_states, next_actions);
    return vector<Status>(env->num_of_agents, Status::SUCCESS);
  }
};

class ProbabilisticSimulator : ActionSimulator {
public:
  ProbabilisticSimulator(float success_chance, ActionModelWithRotate model)
      : success_chance_(success_chance), rd_(), gen_(rd_()), distrib_(0, 1),
        ActionSimulator(model){};

  vector<Status> simulate_action(SharedEnvironment *env,
                                 vector<Action> &next_actions) {
    vector<Status> progress(env->num_of_agents);
    for (int i = 0; i < env->num_of_agents; i++) {
      // Succeeds success_chance % of the time, never if <=0 and always if >=1
      if (success_chance_ > distrib_(gen_)) {
        progress[i] = Status::SUCCESS;
        State &curr = env->curr_states.at(i);
        curr = model_.result_state(curr, next_actions.at(i));
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

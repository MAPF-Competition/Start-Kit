
#include "ActionModel.h"
#include "ExecutionSimulator.h"
#include "ScheduleTable.hpp"
#include <deque>
#include <vector>

class BaseCoordinator {
public:
  BaseCoordinator(int batch_length, int num_agents, ActionExecutor &executor,
                  ActionModelWithRotate &model)
      : batch_length_(batch_length), num_agents_(num_agents),
        executor_(executor),
        model_(model) {}

  // Send an action for all agents to the Coordinator
  virtual void send_action(vector<Action> &next_action);

  // Simulates all queued actions
  virtual vector<State> do_action(const vector<State> &curr_states);

  // Returns true if after updating the Coordinator wants to replan instead of repair 
  virtual bool update_needs_replan(const vector<bool> &successed);

  inline int get_batch_size() { return batch_length_; }

protected:
  int batch_length_;
  int num_agents_;
  ActionExecutor &executor_;
  ActionModelWithRotate &model_;
};

class SimpleCoordinator : BaseCoordinator {
public:
  SimpleCoordinator(int batch_length, int num_agents, ActionExecutor &executor,
                    ActionModelWithRotate &model)
      : action_queues_(vector<deque<Action>>(num_agents)), BaseCoordinator(batch_length, num_agents, executor, model){};

  void send_action(vector<Action> &next_action) override {
    for (int i = 0; i < num_agents_; i++) {
      action_queues_.at(i).push_back(next_action.at(i));
    }
    return;
  }

  
  vector<State> do_action(const vector<State> &curr_states) override {
    vector<State> next_states(num_agents_);
    for (int i = 0; i < num_agents_; i++) {
      next_states.at(i) = model_.result_state(curr_states.at(i), action_queues_.at(i).front());
    }
    return next_states;
  }

// Returns true if the plan fails any action and forces a replan 
  bool update_needs_replan(const vector<bool> &successed) override {
    for (int i = 0; i < num_agents_; i++) {
      if (successed.at(i)) {
        action_queues_.at(i).pop_front();
      } else {
        return true;
      }
    }
    return false;
  }


private:
vector<deque<Action>> action_queues_;
};


class MCPCoordinator : BaseCoordinator {
public:
  MCPCoordinator(int batch_length, int num_agents, ActionExecutor &executor,
                 ActionModelWithRotate &model)
      : schedule_(),
        BaseCoordinator(batch_length, num_agents, executor, model){};

  vector<State> do_actions(vector<State> &curr_states,
                           vector<std::deque<Action>> &next_actions) {
    agent_states_.resize(curr_states.size());
    // Schedule beginning location
    schedule(curr_states, next_actions);

    for (int agents_done = 0; agents_done < curr_states.size();) {
      vector<State> next_states(curr_states.size());
      for (int i = 0; i < curr_states.size(); i++) {
        State next_state = model_.result_state(agent_states_.at(i),
                                               next_actions.at(i).front());

        // If scheduled: Move Else: Wait
        if (schedule_.is_scheduled(i, next_state.location)) {
          next_states.at(i) = next_state;
        } else {
          next_states.at(i) = agent_states_.at(i);
        }
      }

      executor_.send_plan(agent_states_, next_states);

      vector<bool> successes = executor_.get_agent_success(0);
      agent_states_ = executor_.get_agent_locations(0);
      for (int i = 0; i < successes.size(); i++) {
        if (successes[i]) {
        }
      }
    }

    return agent_states_;
  }

private:
  ScheduleTable schedule_;
  vector<State> agent_states_;

  void schedule(vector<State> curr_states,
                vector<std::deque<Action>> &next_actions) {
    schedule_.insert_step(curr_states);
    agent_states_ = curr_states;
    vector<Action> next_action(next_actions.size());

    for (int i = 0; i < next_actions.size(); i++) {
      std::deque<Action> &agent_action = next_actions.at(i);
      State curr_state = curr_states.at(i);

      for (Action act : agent_action) {
        State next_state = model_.result_state(curr_state, act);
        schedule_.insert_one(i, next_state.location);
        curr_state = next_state;
      }
    }
    return;
  }
};

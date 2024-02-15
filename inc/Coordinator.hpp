
#include "ActionModel.h"
#include "ExecutionSimulator.h"
#include "ScheduleTable.hpp"
#include <deque>

class BaseCoordinator {
public:
  BaseCoordinator(int batch_length, int num_agents, ActionExecutor &executor,
                  ActionModelWithRotate &model)
      : batch_length_(batch_length), num_agents_(num_agents),
        executor_(executor), model_(model) {}

  // Send an *n* actions for all agents to the Coordinator
  // Laid out as first index is nth action, then second index is agent_id
  // As we want to access a single timestep for every agent
  virtual void send_actions(vector<vector<Action>> &next_action,
                            const vector<State> &start_states);

  // Get next action after processing
  virtual const vector<Action> get_next_actions();

  // Returns true if after updating the Coordinator wants to replan instead of
  // repair
  virtual bool update_needs_replan(const vector<bool> &succeeded);

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
      : BaseCoordinator(batch_length, num_agents, executor, model){};
  
};
class MCPCoordinator : BaseCoordinator {
public:
  MCPCoordinator(int batch_length, int num_agents, ActionExecutor &executor,
                 ActionModelWithRotate &model)
      : schedule_(),
        BaseCoordinator(batch_length, num_agents, executor, model){};

  void send_actions(vector<vector<Action>> &next_action,
                    const vector<State> &start_states) override {
    curr_states_ = start_states;
    for (vector<Action> nth_action : next_action) {
      for (int i = 0; i < num_agents_; i++) {
        action_queues_.at(i).push_back(nth_action.at(i));
      }
    }
    return;
  }

  const vector<Action> get_next_actions() override {
    // Schedule beginning location

    vector<Action> next_actions(num_agents_);

    for (int i = 0; i < num_agents_; i++) {
      State next_state = get_next_state(i, curr_states_.at(i));

      // If scheduled: Move Else: Wait
      if (schedule_.is_scheduled(i, next_state.location)) {
        next_actions.at(i) = get_next_action(i);
      } else {
        next_actions.at(i) = Action::W; 
      }
    }
    curr_actions_ = next_actions;
    return next_actions;
  }

  bool update_needs_replan(const vector<bool> &succeeded ) override {
    bool all_complete = true;
    for (int i = 0; i < num_agents_; i++) {
      deque<Action> agent_actions = action_queues_.at(i);

      // No update if agent's actions are completed
      if (agent_actions.empty()) {
        continue;
      } 
      all_complete = false;
      // If the next action was not delayed with a WAIT, (Wait is never delayed by wait)
      if (curr_actions_.at(i) == Action::W && agent_actions.front() != Action::W) {
        continue;
      }

      // Agent was not delayed abd completed action, clear schedule and advance location
      schedule_.pop_entry(i, curr_states_.at(i).location);
      curr_states_.at(i) = model_.result_state(curr_states_.at(i), agent_actions.front());

    }
    return all_complete;
  }

private:
  ScheduleTable schedule_;
  vector<State> curr_states_;
  vector<deque<Action>> action_queues_;
  vector<Action> curr_actions_;

  void schedule(vector<State> curr_states,
                vector<std::deque<Action>> &next_actions) {
    schedule_.insert_step(curr_states);
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

  vector<State> get_next_states(const vector<State> &curr_state) {
    vector<State> next_state(num_agents_);
    for (int i = 0; i < num_agents_; i++) {
      next_state[i] =
          model_.result_state(curr_state.at(i), action_queues_.at(i).front());
    }
    return next_state;
  }

  // Returns next queued action, or Wait if no actions queued
  inline Action get_next_action(int agent_id) {
    deque<Action> next_actions = action_queues_.at(agent_id);
    return next_actions.empty() ? Action::W : next_actions.front();
  }

  inline State get_next_state(int agent_id, const State curr_state) {
    return model_.result_state(curr_state, get_next_action(agent_id));
  }
};

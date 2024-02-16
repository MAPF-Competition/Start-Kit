#include "ActionModel.h"
#include <cstddef>
#include <vector>

struct metrics_t {
  size_t num_queries_;
  double seconds_;

  metrics_t() : num_queries_(0), seconds_(0) {}
};

template <class P> class PlannerWrapper {
public:
  PlannerWrapper(P *planner) : planner_(planner) {}

  // Calls the planner's query method and updates the metrics
  // @param start The start states of the agents
  // @param goal The goal states of the agents
  // @return The next action for each agent
  std::vector<Action> &query(const std::vector<size_t> &starts,
                             const std::vector<size_t> &goals,
                             double time_limit = 0.0) {

    // TODO: should implement a timer
    metrics_.num_queries_++;
    auto start_time = std::chrono::steady_clock::now();
    std::vector<Action> &actions = planner_->query(starts, goals);
    metrics_.seconds_ +=
        std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::steady_clock::now() - start_time)
            .count();

    return actions;
  }

  // Returns the metrics of the planner
  const metrics_t &get_metrics() const { return metrics_; }

private:
  P *planner_;
  metrics_t metrics_;
};

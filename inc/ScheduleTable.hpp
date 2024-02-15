#include "States.h"
#include <queue>
#include <unordered_map>

typedef int agent_id;

class ScheduleTable {
public:
  ScheduleTable() {}

  void insert_series(int agent_id, vector<State> &locations) {
    for (State place : locations) {
      this->insert_one(agent_id, place.location);
    }
    return;
  }

  void insert_step(vector<State> &locations) {
    for (int i = 0; i < locations.size(); i++) {
      this->insert_one(i, locations.at(i).location);
    }
    return;
  }

  bool is_scheduled(int agent_id, int location) {
    auto search = schedule_.find(location);
    // Check if key is in map
    if (search == schedule_.end()) {
      assert(false); // Tried to lookup unplanned location
      return false;
    }
    // If the queried agent is first in line for the location
    if (search->second.front() == agent_id) {
      return true;
    }
    // Not first in line
    return false;
  }
  // I might want to validate this...?
  void pop_entry(int agent_id, int location) {
    auto search = schedule_.find(location);

    if (search == schedule_.end()) {
      assert(false); // Location was never planned
      return;
    }

    if (search->second.front() != agent_id) {
      assert(false); // Agent id is not first in line
      return;
    }

    search->second.pop();
    return;
  }

  inline void insert_one(int agent_id, int location) {
    auto node = schedule_.find(location);
    if (node == schedule_.end()) {
      schedule_.insert({location, std::queue<int>({agent_id})});
    } else {
      node->second.push(agent_id);
    }
    return;
  }

private:
  std::unordered_map<int, std::queue<agent_id>> schedule_;
};

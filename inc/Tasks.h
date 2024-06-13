#pragma once
#include "common.h"

struct Task
{
    int task_id;
    // int location;
    int t_assigned = -1;
    int t_completed = -1;
    int t_revealed = -1;
    int agent_assigned = -1;

    vector<int> locations;
    int idx_next_loc = 0;

  int get_next_loc(){
    if (idx_next_loc < locations.size()){
      return locations.at(idx_next_loc);
    } else {
      assert(false);
    }
  }

  bool is_finished(){
    return idx_next_loc == locations.size();
  }

  Task(int task_id, int location): task_id(task_id), locations({location}) {};
    Task(int task_id, int location, int t_assigned, int agent_assigned):
      task_id(task_id), locations({location}),
        t_assigned(t_assigned), agent_assigned(agent_assigned)
    {};
};

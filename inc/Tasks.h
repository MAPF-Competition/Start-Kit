#pragma once
#include "common.h"

struct Task
{
    int task_id;
    int t_completed = -1;
    int t_revealed = -1;
    int agent_assigned = -1;

    vector<int> locations;
    int idx_next_loc = 0;




    int get_next_loc()
    {
        if (idx_next_loc < locations.size())
        {
            return locations.at(idx_next_loc);
        } 
        else 
        {
            assert(false);
            return -1;
        }
    }

    bool is_finished()
    {
        return idx_next_loc == locations.size();
    }

    //Task(int task_id, int location): task_id(task_id), locations({location}) {};
    Task(int task_id, list<int> location, int t_revealed): task_id(task_id), t_revealed(t_revealed)
    {
        for (auto loc: location)
            locations.push_back(loc);
    };

    Task(){};

    Task(Task* other)
    {
        task_id = other->task_id;
        t_completed = other->t_completed;
        locations = other->locations;
        t_revealed = other->t_revealed;
        idx_next_loc = other->idx_next_loc;
        agent_assigned = other->agent_assigned;
    };

    Task(const Task& other)
    {
        task_id = other.task_id;
        t_completed = other.t_completed;
        locations = other.locations;
        t_revealed = other.t_revealed;
        idx_next_loc = other.idx_next_loc;
        agent_assigned = other.agent_assigned;
    };
};

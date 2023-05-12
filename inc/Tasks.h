#pragma once
#include "common.h"

struct Task
{
    int task_id;
    int location;
    int t_assigned = -1;
    int t_completed = -1;
    int agent_assigned = -1;


    Task(int task_id, int location): task_id(task_id), location(location) {};
    Task(int task_id, int location, int t_assigned, int agent_assigned):
        task_id(task_id), location(location),
        t_assigned(t_assigned), agent_assigned(agent_assigned)
    {};
};

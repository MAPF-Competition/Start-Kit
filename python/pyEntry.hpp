#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include <pybind11/numpy.h>
#include "SharedEnv.h"
#include "States.h"
#include "Entry.h"
#include "pyMAPFPlanner.hpp"
#include "pyTaskScheduler.hpp"

class PyEntry : public Entry
{
public:
    PyEntry(SharedEnvironment *env) : Entry(env){
        planner=new pyMAPFPlanner(env);
        scheduler= new pyTaskScheduler(env);
    }
    PyEntry()
    {
        env = new SharedEnvironment();
        planner = new pyMAPFPlanner(env);
        scheduler = new pyTaskScheduler(env);
    }

    void initialize(int preprocess_time_limit)
    {
        planner->initialize(preprocess_time_limit);
    }

    void compute(int time_limit, std::vector<Action> &plan, std::vector<int> &proposed_schedule)
    {

        // first call task schedule
        scheduler->plan(time_limit, proposed_schedule);

        // then update the task location to planner
        for (size_t i = 0; i < proposed_schedule.size(); i++)
        {
            env->goal_locations[i].clear();
            int t_id = proposed_schedule[i];
            if (t_id == -1)
                continue;
            Task *task_ptr = nullptr;
            for (Task &task : env->task_pool)
            {
                if (task.task_id == t_id)
                {
                    task_ptr = &task;
                    break;
                }
            }
            if (task_ptr == nullptr)
            {
                continue;
            }
            int i_loc = task_ptr->idx_next_loc;
            env->goal_locations[i].push_back({task_ptr->locations.at(i_loc), task_ptr->t_revealed});
        }

        // then call planner
        planner->plan(time_limit, plan);
    }
};
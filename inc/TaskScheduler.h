#pragma once
#include "Tasks.h"
#include "SharedEnv.h"


class TaskScheduler
{
    public:
        SharedEnvironment* env;

        TaskScheduler(SharedEnvironment* env): env(env){};
        TaskScheduler(){env = new SharedEnvironment();};
        virtual ~TaskScheduler(){delete env;};
        virtual void initialize(int preprocess_time_limit);
        virtual void plan(int time_limit, std::vector<int> & proposed_schedule);
};
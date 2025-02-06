/**
 * @file MAPFPlanner.h
 * @brief header file for MAPFPlanner.cpp.
 * 
 * @authors Zhe Chen Han Zhang, Yue Zhang
 * 
 * @note All authors contributed equally to this work.
 */

#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"


class MAPFPlanner
{
public:
    SharedEnvironment* env;

	MAPFPlanner(SharedEnvironment* env): env(env){};
    MAPFPlanner(){env = new SharedEnvironment();};
	virtual ~MAPFPlanner(){delete env;};


    virtual void initialize(int preprocess_time_limit);

    // return next states for all agents
    virtual void plan(int time_limit, std::vector<Action> & plan);

};

/**
 * pyEntry.hpp
 * Entry subclass that can mix Python and C++ components for planner, scheduler, and executor.
 * 
 * Component selection is driven by three booleans:
 *   - plannerPython:   true → use Python MAPFPlanner, false → use C++ default
 *   - schedulerPython: true → use Python TaskScheduler, false → use C++ default
 *   - executorPython:  true → use Python Executor, false → use C++ default Executor
 *
 * This allows any combination of Python/C++ for different competition tracks.
 */
#pragma once
#include "Entry.h"
#include "Executor.h"
#include "pyMAPFPlanner.hpp"
#include "pyTaskScheduler.hpp"
#include "pyExecutor.hpp"

class pyEntry : public Entry
{
public:
    bool use_py_planner;
    bool use_py_scheduler;
    bool use_py_executor;

    Executor* executor_ptr = nullptr; // stored so driver.cpp can pass it to Simulator

    pyEntry(bool plannerPython, bool schedulerPython, bool executorPython)
        : Entry(), use_py_planner(plannerPython), use_py_scheduler(schedulerPython), use_py_executor(executorPython)
    {
        // Entry() already created default env, planner, scheduler.
        // Replace components with Python bridges as needed.
        // IMPORTANT: Must nullify env pointers before deleting defaults,
        // because MAPFPlanner/TaskScheduler destructors delete env.

        if (use_py_planner) {
            planner->env = nullptr; // prevent ~MAPFPlanner from deleting shared env
            delete planner;
            planner = new pyMAPFPlanner(env);
            // Prevent the new planner's destructor from deleting shared env too
            // (pyMAPFPlanner inherits MAPFPlanner which deletes env in dtor)
        }
        if (use_py_scheduler) {
            scheduler->env = nullptr; // prevent ~TaskScheduler from deleting shared env
            delete scheduler;
            scheduler = new pyTaskScheduler(env);
        }
        if (use_py_executor) {
            executor_ptr = new pyExecutor();
        } else {
            executor_ptr = new Executor();
        }
    }

    ~pyEntry() override
    {
        // Prevent planner/scheduler destructors from double-deleting env.
        // Entry::~Entry() handles env deletion.
        if (planner) { planner->env = nullptr; delete planner; planner = nullptr; }
        if (scheduler) { scheduler->env = nullptr; delete scheduler; scheduler = nullptr; }
        if (executor_ptr) { executor_ptr->env = nullptr; delete executor_ptr; executor_ptr = nullptr; }
    }

    /// Get the executor pointer (to pass to Simulator). Caller takes ownership.
    Executor* get_executor()
    {
        Executor* ret = executor_ptr;
        executor_ptr = nullptr;
        return ret;
    }
};

#include <cmath>
#include "CompetitionSystem.h"
#include <boost/tokenizer.hpp>
#include "SharedEnv.h"
#include "nlohmann/json.hpp"
#include <functional>
#include <Logger.h>

using json = nlohmann::ordered_json;


void BaseSystem::move(vector<Action>& actions)
{

    vector<State> curr_states = simulator.move(actions);
    int timestep = simulator.get_curr_timestep();
    // agents do not move

    for (int k = 0; k < num_of_agents; k++)
    {
        paths[k].push_back(curr_states[k]);
        actual_movements[k].push_back(actions[k]);
    }
}




// // This function might not work correctly with small map (w or h <=2)
// bool BaseSystem::valid_moves(vector<State>& prev, vector<Action>& action)
// {
//   return model->is_valid(prev, action);
// }


void BaseSystem::sync_shared_env() {

  if (!started){
      env->goal_locations.resize(num_of_agents);
      task_manager.sync_shared_env(env);
      simulator.sync_shared_env(env);
  }
  else
  {
    env->curr_timestep = simulator.get_curr_timestep();
  }
}


std::pair<vector<Action> ,vector<int> > BaseSystem::plan_wrapper()
{
    vector<Action> actions;
    vector<int> proposed_schedule;
    planner->compute(plan_time_limit, actions,proposed_schedule);
    return {actions, proposed_schedule};
}


void BaseSystem::plan(vector<Action> & actions,vector<int> & proposed_schedule)
{

    int timestep = simulator.get_curr_timestep();

    using namespace std::placeholders;
    if (started && future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
        std::cout << started << "     " << (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) << std::endl;
        if(logger)
        {
            logger->log_info("planner cannot run because the previous run is still running", timestep);
        }

        if (future.wait_for(std::chrono::milliseconds(plan_time_limit)) == std::future_status::ready)
        {
            task_td.join();
            started = false;
            auto res = future.get();
            actions = res.first;
            proposed_schedule = res.second;
            return;
        }
        logger->log_info("planner timeout", timestep);
        return;
    }

    std::packaged_task<std::pair<vector<Action> ,vector<int> >()> task(std::bind(&BaseSystem::plan_wrapper, this));
    future = task.get_future();
    if (task_td.joinable())
    {
        task_td.join();
    }
    env->plan_start_time = std::chrono::steady_clock::now();
    task_td = std::thread(std::move(task));
    started = true;
    if (future.wait_for(std::chrono::milliseconds(plan_time_limit)) == std::future_status::ready)
    {
        task_td.join();
        started = false;
        auto res = future.get();
        actions = res.first;
        proposed_schedule = res.second;

        return;
    }
    logger->log_info("planner timeout", timestep);
    return;
}


bool BaseSystem::planner_initialize()
{
    using namespace std::placeholders;
    std::packaged_task<void(int)> init_task(std::bind(&Entry::initialize, planner, _1));
    auto init_future = init_task.get_future();
    
    env->plan_start_time = std::chrono::steady_clock::now();
    auto init_td = std::thread(std::move(init_task), preprocess_time_limit);
    if (init_future.wait_for(std::chrono::milliseconds(preprocess_time_limit)) == std::future_status::ready)
    {
        init_td.join();
        return true;
    }

    init_td.detach();
    return false;
}


void BaseSystem::log_preprocessing(bool succ)
{
    if (logger == nullptr)
        return;
    if (succ)
    {
        logger->log_info("Preprocessing success", simulator.get_curr_timestep());
    } 
    else
    {
        logger->log_fatal("Preprocessing timeout", simulator.get_curr_timestep());
    }
    logger->flush();
}

// void BaseSystem::log_event_assigned(int agent_id, int task_id, int timestep)
// {
//     logger->log_info("Task " + std::to_string(task_id) + " is assigned to agent " + std::to_string(agent_id), timestep);
// }


// Moved to TaskManager
// void BaseSystem::log_event_finished(int agent_id, int task_id, int timestep) 
// {
//     logger->log_info("Agent " + std::to_string(agent_id) + " finishes task " + std::to_string(task_id), timestep);
// }


void BaseSystem::simulate(int simulation_time)
{
    //init logger
    //Logger* log = new Logger();
    initialize();

    for (; simulator.get_curr_timestep() < simulation_time; )
    {
        // find a plan
        sync_shared_env();

        auto start = std::chrono::steady_clock::now();

        vector<Action> actions;
        vector<int> proposed_schedule;
        plan(actions,proposed_schedule);

        auto end = std::chrono::steady_clock::now();

        for (int a = 0; a < num_of_agents; a++)
        {
            if (!env->goal_locations[a].empty())
                solution_costs[a]++;
        }

        // move drives
        //move(actions);
        vector<State> curr_states = simulator.move(actions);
        int timestep = simulator.get_curr_timestep();
        // agents do not move

        for (int k = 0; k < num_of_agents; k++)
        {
            paths[k].push_back(curr_states[k]);
            actual_movements[k].push_back(actions[k]);
        }
        if (!planner_movements[0].empty() && planner_movements[0].back() == Action::NA)
        {
            planner_times.back()+=plan_time_limit;  //add planning time to last record
        }
        else
        {
            auto diff = end-start;
            planner_times.push_back(std::chrono::duration<double>(diff).count());
        }

        // update tasks
        task_manager.update_tasks(curr_states, proposed_schedule, simulator.get_curr_timestep());
    }
}


void BaseSystem::initialize()
{
    env->num_of_agents = num_of_agents;
    env->rows = map.rows;
    env->cols = map.cols;
    env->map = map.map;

    
    // // bool succ = load_records(); // continue simulating from the records
    // timestep = 0;
    // curr_states = starts;

    int timestep = simulator.get_curr_timestep();

    //planner initilise before knowing the first goals
    bool planner_initialize_success= planner_initialize();
    
    log_preprocessing(planner_initialize_success);
    if (!planner_initialize_success)
        _exit(124);

    // initialize_goal_locations();
    task_manager.reveal_tasks(timestep);

    sync_shared_env();

    actual_movements.resize(num_of_agents);
    planner_movements.resize(num_of_agents);
    solution_costs.resize(num_of_agents);
    for (int a = 0; a < num_of_agents; a++)
    {
        solution_costs[a] = 0;
    }
}


void BaseSystem::saveResults(const string &fileName, int screen) const
{
    json js;
    // Save action model
    js["actionModel"] = "MAPF_T";
    js["version"] = "2024 LoRR";

    std::string feasible = fast_mover_feasible ? "Yes" : "No";
    js["AllValid"] = feasible;

    js["teamSize"] = num_of_agents;

    js["numTaskFinished"] = task_manager.num_of_task_finish;
    int sum_of_cost = 0;
    int makespan = 0;
    if (num_of_agents > 0)
    {
        sum_of_cost = solution_costs[0];
        makespan = solution_costs[0];
        for (int a = 1; a < num_of_agents; a++)
        {
            sum_of_cost += solution_costs[a];
            if (solution_costs[a] > makespan)
            {
                makespan = solution_costs[a];
            }
        }
    }
    js["sumOfCost"] = sum_of_cost;
    js["makespan"] = makespan;

    // Save start locations[x,y,orientation]
    if (screen <= 2)
    {
        js["start"] = simulator.starts_to_json();
    }
    
    if (screen <= 2)
    {
        js["actualPaths"] = simulator.actual_path_to_json();
    }

    if (screen <=1)
    {
        js["plannerPaths"] = simulator.planned_path_to_json();

        json planning_times = json::array();
        for (double time: planner_times)
            planning_times.push_back(time);
        js["plannerTimes"] = planning_times;

        // Save errors
        js["errors"] = simulator.action_errors_to_json();

        //actual schedules
        json aschedules = json::array();
        for (int i = 0; i < num_of_agents; i++)
        {
            std::string schedules;
            bool first = true;
            for (const auto schedule : task_manager.actual_schedule[i])
            {
                if (!first)
                {
                    schedules+= ",";
                } 
                else 
                {
                    first = false;
                }

                schedules+=std::to_string(schedule.first);
                schedules+=":";
                int tid = schedule.second;
                schedules+=std::to_string(tid);
            }  
            aschedules.push_back(schedules);
        }

        js["actualSchedule"] = aschedules;

        //planned schedules
        json pschedules = json::array();
        for (int i = 0; i < num_of_agents; i++)
        {
            std::string schedules;
            bool first = true;
            for (const auto schedule : task_manager.planner_schedule[i])
            {
                if (!first)
                {
                    schedules+= ",";
                } 
                else 
                {
                    first = false;
                }

                schedules+=std::to_string(schedule.first);
                schedules+=":";
                int tid = schedule.second;
                schedules+=std::to_string(tid);
                
            }  
            pschedules.push_back(schedules);
        }

        js["plannerSchedule"] = pschedules;

        // Save errors
        json schedule_errors = json::array();
        for (auto error: task_manager.schedule_errors)
        {
            std::string error_msg;
            int t_id;
            int agent1;
            int agent2;
            int timestep;
            std::tie(error_msg,t_id,agent1,agent2,timestep) = error;
            json e = json::array();
            e.push_back(t_id);
            e.push_back(agent1);
            e.push_back(agent2);
            e.push_back(timestep);
            e.push_back(error_msg);
            schedule_errors.push_back(e);
        }

        js["scheduleErrors"] = schedule_errors;

        // Save events
        json event = json::array();
        for(auto e: task_manager.events)
        {
            json ev = json::array();
            int timestep;
            int agent_id;
            int task_id;
            int seq_id;
            std::tie(timestep,agent_id,task_id,seq_id) = e;
            ev.push_back(timestep);
            ev.push_back(agent_id);
            ev.push_back(task_id);
            ev.push_back(seq_id);
            event.push_back(ev);
        }
        js["events"] = event;

        // Save all tasks
        json tasks = task_manager.to_json(map.cols);
        js["tasks"] = tasks;
    }

    std::ofstream f(fileName,std::ios_base::trunc |std::ios_base::out);
    f << std::setw(4) << js;

}



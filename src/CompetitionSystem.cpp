#include <cmath>
#include "CompetitionSystem.h"
#include <boost/tokenizer.hpp>
#include "SharedEnv.h"
#include "nlohmann/json.hpp"
#include <functional>
#include <Logger.h>

using json = nlohmann::ordered_json;


list<Task> BaseSystem::move(vector<Action>& actions)
{

    vector<State> curr_states = simulator.move(actions);
    int timestep = simulator.get_curr_timestep();
    // agents do not move

    for (int k = 0; k < num_of_agents; k++){
        paths[k].push_back(curr_states[k]);
        actual_movements[k].push_back(actions[k]);
    }
    return task_manager.check_finished_tasks(curr_states, timestep);
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


vector<Action> BaseSystem::plan_wrapper()
{
    vector<Action> actions;
    planner->plan(plan_time_limit, actions);

    return actions;
}


vector<Action> BaseSystem::plan()
{

    int timestep = simulator.get_curr_timestep();

    vector<Action> actions;
    planner->plan(plan_time_limit, actions);

    return actions;

    using namespace std::placeholders;
    if (started && future.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
    {
        std::cout << started << "     " << (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) << std::endl;
        if(logger)
        {
            logger->log_info("planner cannot run because the previous run is still running", timestep);
        }

        if (future.wait_for(std::chrono::seconds(plan_time_limit)) == std::future_status::ready)
        {
            task_td.join();
            started = false;
            return future.get();
        }
        logger->log_info("planner timeout", timestep);
        return {};
    }

    std::packaged_task<std::vector<Action>()> task(std::bind(&BaseSystem::plan_wrapper, this));
    future = task.get_future();
    if (task_td.joinable())
    {
        task_td.join();
    }
    task_td = std::thread(std::move(task));
    started = true;
    if (future.wait_for(std::chrono::seconds(plan_time_limit)) == std::future_status::ready)
    {
        task_td.join();
        started = false;
        return future.get();
    }
    logger->log_info("planner timeout", timestep);
    return {};
}


bool BaseSystem::planner_initialize()
{
    using namespace std::placeholders;
    std::packaged_task<void(int)> init_task(std::bind(&MAPFPlanner::initialize, planner, _1));
    auto init_future = init_task.get_future();
    
    auto init_td = std::thread(std::move(init_task), preprocess_time_limit);
    if (init_future.wait_for(std::chrono::seconds(preprocess_time_limit)) == std::future_status::ready)
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


void BaseSystem::log_event_assigned(int agent_id, int task_id, int timestep)
{
    logger->log_info("Task " + std::to_string(task_id) + " is assigned to agent " + std::to_string(agent_id), timestep);
}


void BaseSystem::log_event_finished(int agent_id, int task_id, int timestep) 
{
    logger->log_info("Agent " + std::to_string(agent_id) + " finishes task " + std::to_string(task_id), timestep);
}


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

        vector<Action> actions = plan();

        auto end = std::chrono::steady_clock::now();

        for (int a = 0; a < num_of_agents; a++)
        {
            if (!env->goal_locations[a].empty())
                solution_costs[a]++;
        }

        // move drives
        list<Task> new_finished_tasks = move(actions);
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

        bool complete_all = task_manager.update_tasks(simulator.get_curr_timestep());

        if (complete_all)
        {
            break;
        }
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
    task_manager.update_tasks(timestep);

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

    std::string feasible = fast_mover_feasible ? "Yes" : "No";
    js["AllValid"] = feasible;

    js["teamSize"] = num_of_agents;

    // Save start locations[x,y,orientation]
    if (screen <= 2)
    {
        js["start"] = simulator.starts_to_json();
    }

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

        // Save events
        json events_json = json::array();
        for (int i = 0; i < num_of_agents; i++)
        {
            json event = json::array();
            for(auto e: events[i])
            {
                json ev = json::array();
                std::string event_msg;
                int task_id;
                int timestep;
                std::tie(task_id,timestep,event_msg) = e;
                ev.push_back(task_id);
                ev.push_back(timestep);
                ev.push_back(event_msg);
                event.push_back(ev);
            }
            events_json.push_back(event);
        }
        js["events"] = events_json;

        // Save all tasks
        json tasks = task_manager.to_json(map.cols);
        js["tasks"] = tasks;
    }

    std::ofstream f(fileName,std::ios_base::trunc |std::ios_base::out);
    f << std::setw(4) << js;

}



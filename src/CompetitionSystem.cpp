#include <cmath>
#include "CompetitionSystem.h"
#include <boost/tokenizer.hpp>
#include "SharedEnv.h"
#include "nlohmann/json.hpp"
#include <functional>
#include <Logger.h>
#include <fstream>

using json = nlohmann::ordered_json;

void BaseSystem::set_task_trend_output(const std::string& file_name, int interval)
{
    task_trend_output_file = file_name;
    task_trend_interval = interval;
}

void BaseSystem::write_task_trend_snapshot(int timestep, bool force)
{
    if (task_trend_output_file.empty())
    {
        return;
    }

    if (!force)
    {
        if (task_trend_interval <= 0 || timestep <= 0 || timestep % task_trend_interval != 0)
        {
            return;
        }
    }

    if (timestep == last_task_trend_timestep)
    {
        return;
    }

    std::ofstream out(task_trend_output_file, std::ios::app);
    if (!out.is_open())
    {
        if (logger != nullptr)
        {
            logger->log_warning("Failed to open task trend output file: " + task_trend_output_file, timestep);
        }
        return;
    }

    const int cumulative_finished = task_manager.num_of_task_finish;
    const int interval_finished = cumulative_finished - last_task_trend_finished;
    out << timestep << " " << cumulative_finished << " " << interval_finished << "\n";

    last_task_trend_timestep = timestep;
    last_task_trend_finished = cumulative_finished;
}

void BaseSystem::sync_shared_env_planner() 
{
    env->goal_locations.resize(num_of_agents);
    task_manager.sync_shared_env(env);
    simulator.sync_shared_env(env);
}

void BaseSystem::sync_shared_env_executor() 
{
    simulator.sync_shared_env(exec_env);
}


bool BaseSystem::planner_wrapper()
{
    planner->compute(min_comm_time, proposed_plan, proposed_schedule);
    return true;
}

bool BaseSystem::planner_wrapper_init()
{
    planner->compute(initial_plan_time_limit, proposed_plan, proposed_schedule);
    return true;
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


void BaseSystem::simulate(int simulation_time, int chunk_size)
{
    simulator.set_chunk(chunk_size, simulation_time);
    initialize();

    this->simulation_time = simulation_time;

    vector<State> curr_states = simulator.get_current_state();

    //start initial planning
    plan_time_limit = initial_plan_time_limit;
    std::packaged_task<bool()> task(std::bind(&BaseSystem::planner_wrapper_init, this));
    future = task.get_future();
    env->plan_start_time = std::chrono::steady_clock::now();
    task_td = std::thread(std::move(task));
    started = true;
    task_manager.clear_new_agents_tasks();

    if (future.wait_for(std::chrono::milliseconds(initial_plan_time_limit)) == std::future_status::ready)
    {
        task_td.join();
        started = false;
        auto res = future.get();
        logger->log_info("planner returns", simulator.get_curr_timestep());
    }
    else
    {
        logger->log_info("planner timeout", simulator.get_curr_timestep());
    }

    //initial planning timeout
    while (started)
    {
        //wait for initial planning to finish and at the same time move all wait
        logger->log_info("planner (initilal planning) cannot run because the previous run is still running", simulator.get_curr_timestep());
        auto deadline   = std::chrono::steady_clock::now() + std::chrono::milliseconds(simulator_time_limit);
        //main thread move drives by calling simulator.move
        simulator.move_all_wait(1);
        auto move_end = std::chrono::steady_clock::now();
        while(deadline < move_end)
        {
            //move takes more time than simulator_time_limit, extend deadline
            deadline += std::chrono::milliseconds(simulator_time_limit);
        }
        // wait until deadline OR planner finishes early
        const auto st = future.wait_until(deadline);

        if (st == std::future_status::ready) 
        {
            task_td.join();
            started = false;
            auto res = future.get();
            logger->log_info("planner (initilal planning) returns", simulator.get_curr_timestep());
        } 
        else 
        {
            logger->log_info("planner (initilal planning) timeout", simulator.get_curr_timestep());
        }
    }

    std::chrono::steady_clock::time_point plan_start = std::chrono::steady_clock::now();

    int remain_communication_time = 0;

    while (simulator.get_curr_timestep() < simulation_time)
    {
        //check if planenr finished
        if (remain_communication_time <= 0 && started)
        {
            auto deadline = plan_start + std::chrono::milliseconds(min_comm_time - remain_communication_time);
            const auto st = future.wait_until(deadline);
            if (st == std::future_status::ready) 
            {
                task_td.join();
                started = false;
                auto res = future.get();
                logger->log_info("planner returns", simulator.get_curr_timestep());
            } 
            else 
            {
                logger->log_info("planner timeout", simulator.get_curr_timestep());
            }
        }

        //planner finished and min communication time reached, launch new planning
        if (!started && remain_communication_time <= 0)
        {
            //apply proposed schedule to task_manager before syncing,
            //so that env->curr_task_schedule reflects the latest accepted assignments.
            task_manager.set_task_assignment(proposed_schedule,simulator.get_curr_timestep());

            //process new plan in simulator
            sync_shared_env_executor();
            simulator.process_new_plan(process_new_plan_time_limit, simulator_time_limit, proposed_plan);

            //launch new planning task
            sync_shared_env_planner();
            plan_time_limit = min_comm_time;
            std::packaged_task<bool()> task(std::bind(&BaseSystem::planner_wrapper, this));
            future = task.get_future();
            env->plan_start_time = std::chrono::steady_clock::now();
            task_td = std::thread(std::move(task));
            started = true;
            plan_start = std::chrono::steady_clock::now();
            task_manager.clear_new_agents_tasks();
            remain_communication_time = min_comm_time;
        }

        //while the planner is running, move from previous plans
        sync_shared_env_executor();
        auto move_start = std::chrono::steady_clock::now();
        curr_states = simulator.move(simulator_time_limit);
        auto move_end = std::chrono::steady_clock::now();

        int elapsed_tick =std::max(1, ((int)std::chrono::duration_cast<std::chrono::milliseconds>(move_end - move_start).count() + simulator_time_limit - 1) / simulator_time_limit);
        remain_communication_time -= elapsed_tick*simulator_time_limit;

        //update tasks
        task_manager.update_tasks(curr_states, proposed_schedule, simulator.get_curr_timestep());
        write_task_trend_snapshot(simulator.get_curr_timestep());
    }

    write_task_trend_snapshot(simulator.get_curr_timestep(), true);
}


void BaseSystem::initialize()
{
    last_task_trend_timestep = 0;
    last_task_trend_finished = 0;

    if (!task_trend_output_file.empty())
    {
        std::ofstream out(task_trend_output_file, std::ios::trunc);
        if (!out.is_open())
        {
            if (logger != nullptr)
            {
                logger->log_warning("Failed to initialize task trend output file: " + task_trend_output_file);
            }
        }
        else
        {
            out << "timestep cumulative_finished interval_finished\n";
        }
    }

    env->num_of_agents = num_of_agents;
    env->rows = map.rows;
    env->cols = map.cols;
    env->map = map.map;

    env->min_planner_communication_time = min_comm_time;
    env->action_time = simulator_time_limit;
    env->max_counter = simulator.get_max_counter();

    exec_env->num_of_agents = num_of_agents;
    exec_env->rows = map.rows;
    exec_env->cols = map.cols;
    exec_env->map = map.map;    

    exec_env->min_planner_communication_time = min_comm_time;
    exec_env->action_time = simulator_time_limit;
    exec_env->max_counter = simulator.get_max_counter();

    int timestep = simulator.get_curr_timestep();

    std::packaged_task<void(int)> init_task(std::bind(&Entry::initialize, planner, std::placeholders::_1));
    auto init_future = init_task.get_future();

    auto init_start_time = std::chrono::steady_clock::now();
    env->plan_start_time = init_start_time;
    exec_env->plan_start_time = init_start_time;
    auto init_deadline   = init_start_time + std::chrono::milliseconds(preprocess_time_limit);
    std::thread init_td(std::move(init_task), preprocess_time_limit);

    simulator.initialise_executor(preprocess_time_limit);

    auto init_end_time = std::chrono::steady_clock::now();

    int diff = (int)std::chrono::duration_cast<std::chrono::milliseconds>(init_end_time - init_start_time).count();

    if (init_future.wait_until(init_deadline) == std::future_status::ready && diff <= preprocess_time_limit)
    {
        init_td.join();
        log_preprocessing(true);
    } 
    else 
    {
        init_td.detach();
        log_preprocessing(false);
        _exit(124);
    }

    // initialize_goal_locations();
    task_manager.reveal_tasks(timestep); //this also intialize env->new_tasks

    sync_shared_env_planner(); // sync the new tasks and new_free_agents to the shared environment for the planner to use in the first planning
    sync_shared_env_executor(); // sync the new tasks and new_free_agents to the shared environment for the executor to use in the first move

    env->new_freeagents.reserve(num_of_agents); //new free agents are empty in task_manager on initialization, set it after task_manager sync
    exec_env->new_freeagents.reserve(num_of_agents);
    for (int i = 0; i < num_of_agents; i++)
    {
        env->new_freeagents.push_back(i);
        exec_env->new_freeagents.push_back(i);
    }

    solution_costs.resize(num_of_agents);
    for (int a = 0; a < num_of_agents; a++)
    {
        solution_costs[a] = 0;
    }

    // proposed_actions.resize(num_of_agents, Action::W);
    proposed_schedule.resize(num_of_agents, -1);
}


void BaseSystem::saveResults(const string &fileName, int screen, bool pretty_print) const
{
    json js;
    // Save action model
    js["actionModel"] = "MAPF_T";
    js["version"] = "2026 LoRR";

    js["teamSize"] = num_of_agents;

    js["numTaskFinished"] = task_manager.num_of_task_finish;
    
    js["makespan"] = simulation_time;

    js["numPlannerErrors"] = simulator.get_number_errors();
    js["numScheduleErrors"] = task_manager.get_number_errors();

    js["numEntryTimeouts"] = total_timetous;

    js["agentMaxCounter"] = simulator.get_max_counter();
    js["outputSegmentSize"]=simulator.get_chunk_size();

    // Save start locations[x,y,orientation]
    if (screen <= 2)
    {
        js["delayIntervals"] = simulator.delay_intervals_to_json();
        js["start"] = simulator.starts_to_json();
    }
    
    if (screen <= 2)
    {
        js["actualPaths"] = simulator.actual_path_to_json();
    }

    if (screen <= 2)
    {
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
    }

    std::ofstream f(fileName,std::ios_base::trunc |std::ios_base::out);
    if (pretty_print)
    {
        f << std::setw(4) << js;
    }
    else
    {
        f << js.dump();
    }

}

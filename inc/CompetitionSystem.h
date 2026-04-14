#pragma once
// #include "BasicSystem.h"
#include "SharedEnv.h"
#include "Grid.h"
#include "Tasks.h"
#include "ActionModel.h"
#include "Plan.h"
#include "Entry.h"
#include "Logger.h"
#include "TaskManager.h"
#include "DelayGenerator.h"
#include <pthread.h>
#include <future>
#include "Simulator.h"
#include <memory>
#include <string>

class BaseSystem
{
public:
    Logger* logger = nullptr;

	BaseSystem(Grid &grid, Entry* planner, Executor* executor, std::vector<int>& start_locs, std::vector<list<int>>& tasks, ActionModelWithRotate* model, int max_counter = 10):
      map(grid), planner(planner), env(planner->env), exec_env(executor->env),
      task_manager(tasks, start_locs.size()), simulator(grid,start_locs,model,executor,max_counter)
    {
        num_of_agents = start_locs.size();
        starts.resize(num_of_agents);

        for (size_t i = 0; i < start_locs.size(); i++)
        {
            if (grid.map[start_locs[i]] == 1)
                {
                    cout<<"error: agent "<<i<<"'s start location is an obstacle("<<start_locs[i]<<")"<<endl;
                    exit(0);
                }
            starts[i] = State(start_locs[i], 0, 0);
        }
    };

	virtual ~BaseSystem()
    {
        //safely exit: wait for join the thread then delete planner and exit
        if (started)
        {
            task_td.join();
        }
        if (planner != nullptr)
        {
            delete planner;
        }
        // exec_env is a non-owning alias of executor->env.
        // ~Simulator deletes executor, whose destructor deletes its env.
    };

    void set_num_tasks_reveal(float num){task_manager.set_num_tasks_reveal(num);};
    void set_plan_time_limit(int initial, int comm, int move, int process_new_plan){
        initial_plan_time_limit = initial;
        min_comm_time = comm;
        simulator_time_limit = move;
        process_new_plan_time_limit = process_new_plan;
    };
    void set_preprocess_time_limit(int limit){preprocess_time_limit = limit;};
    void set_log_level(int level){log_level = level;};
    void set_logger(Logger* logger){
        this->logger = logger;
        task_manager.set_logger(logger);
    }

    void set_delay_generator(std::unique_ptr<DelayGenerator> generator)
    {
        simulator.set_delay_generator(std::move(generator));
    }
    void set_staged_action_validation_enabled(bool enabled)
    {
        simulator.set_staged_action_validation_enabled(enabled);
    }

    void simulate(int simulation_time,int chunk_size);
    bool planner_wrapper();
    bool planner_wrapper_init();

    //void saveSimulationIssues(const string &fileName) const;
    void saveResults(const string &fileName, int screen, bool pretty_print = false) const;
    void set_task_trend_output(const std::string& file_name, int interval);


protected:
    Grid map;
    int simulation_time;

    Plan proposed_plan;
    vector<int> proposed_schedule;

    int total_timetous = 0;


    std::future<bool> future;
    std::thread task_td;
    bool started = false;

    Entry* planner;
    SharedEnvironment* env;
    SharedEnvironment* exec_env;

    int preprocess_time_limit=10;

    int plan_time_limit = 0;

    int initial_plan_time_limit = 1000;
    int min_comm_time = 1000;
    int simulator_time_limit = 100;
    int process_new_plan_time_limit = 100;


    vector<State> starts;
    int num_of_agents;

    int log_level = 1;

    // tasks that haven't been finished but have been revealed to agents;

    vector<list<std::tuple<int,int,std::string>>> events;

    //for evaluation
    vector<int> solution_costs;
    list<double> planner_times; 
    bool fast_mover_feasible = true;

    std::string task_trend_output_file;
    int task_trend_interval = 100;
    int last_task_trend_timestep = 0;
    int last_task_trend_finished = 0;


    void initialize();
    bool planner_initialize();
    void write_task_trend_snapshot(int timestep, bool force = false);


    TaskManager task_manager;
    Simulator simulator;
    // deque<Task> task_queue;
    virtual void sync_shared_env_planner();
    virtual void sync_shared_env_executor();

    void move(vector<Action>& actions);
    bool valid_moves(vector<State>& prev, vector<Action>& next);

    void log_preprocessing(bool succ);
    // void log_event_assigned(int agent_id, int task_id, int timestep);
    // void log_event_finished(int agent_id, int task_id, int timestep);

};


// class TaskAssignSystem : public BaseSystem
// {
// public:
// 	TaskAssignSystem(Grid &grid, MAPFPlanner* planner, std::vector<int>& start_locs, std::vector<int>& tasks, ActionModelWithRotate* model):
//         BaseSystem(grid, planner, model)
//     {
//         int task_id = 0;
//         for (auto& task_location: tasks)
//         {
//             all_tasks.emplace_back(task_id++, task_location);
//             task_queue.emplace_back(all_tasks.back().task_id, all_tasks.back().locations.front());
//             //task_queue.emplace_back(task_id++, task_location);
//         }
//         num_of_agents = start_locs.size();
//         starts.resize(num_of_agents);
//         for (size_t i = 0; i < start_locs.size(); i++)
//         {
//             starts[i] = State(start_locs[i], 0, 0);
//         }
//     };

// 	~TaskAssignSystem(){};


// private:
//     deque<Task> task_queue;

// 	void update_tasks();
// };

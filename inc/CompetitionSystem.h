#pragma once
// #include "BasicSystem.h"
#include "SharedEnv.h"
#include "Grid.h"
#include "Tasks.h"
#include "ActionModel.h"
#include "MAPFPlanner.h"
#include "Logger.h"
#include <pthread.h>
#include <future>

class BaseSystem
{
public:
    int num_tasks_reveal = 1;
    Logger* logger = nullptr;

	BaseSystem(Grid &grid, MAPFPlanner* planner, std::vector<int>& start_locs, std::vector<int>& tasks, ActionModelWithRotate* model):
      map(grid), planner(planner), env(planner->env), model(model),
      tasks(tasks)
    {
        num_of_agents = start_locs.size();
        starts.resize(num_of_agents);
        task_counter.resize(num_of_agents,0);
        tasks_size = tasks.size();

        for (size_t i = 0; i < start_locs.size(); i++)
            {
                if (grid.map[start_locs[i]] == 1)
                    {
                        cout<<"error: agent "<<i<<"'s start location is an obstacle("<<start_locs[i]<<")"<<endl;
                        exit(0);
                    }
                starts[i] = State(start_locs[i], 0, 0);
        }

 //        int task_id = 0;
 // for (auto& task_location: tasks)
 //        {
 //            all_tasks.emplace_back(task_id++, task_location);
 //            task_queue.emplace_back(all_tasks.back().task_id, all_tasks.back().locations.front());
 //            //task_queue.emplace_back(task_id++, task_location);
 //        }
 //        num_of_agents = start_locs.size();
 //        starts.resize(num_of_agents);
 //        for (size_t i = 0; i < start_locs.size(); i++)
 //        {
 //            starts[i] = State(start_locs[i], 0, 0);
 //        }
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
    };

    list<Task> check_finished_tasks(vector<State> states);
    void set_num_tasks_reveal(int num){num_tasks_reveal = num;};
    void set_plan_time_limit(int limit){plan_time_limit = limit;};
    void set_preprocess_time_limit(int limit){preprocess_time_limit = limit;};
    void set_logger(Logger* logger){this->logger = logger;}

	void simulate(int simulation_time);
    vector<Action> plan();
    vector<Action> plan_wrapper();

    void savePaths(const string &fileName, int option) const; //option = 0: save actual movement, option = 1: save planner movement
    //void saveSimulationIssues(const string &fileName) const;
    void saveResults(const string &fileName, int screen) const;


protected:
    Grid map;

    std::future<std::vector<Action>> future;
    std::thread task_td;
    bool started = false;

    MAPFPlanner* planner;
    SharedEnvironment* env;

    ActionModelWithRotate* model;

    // #timesteps for simulation
    int timestep;

    int preprocess_time_limit=10;

    int plan_time_limit = 3;

    std::vector<Path> paths;
    std::vector<std::list<Task > > finished_tasks; // location + finish time

    vector<State> starts;
    int num_of_agents;

    vector<State> curr_states;

    vector<list<Action>> actual_movements;
    vector<list<Action>> planner_movements;

    // tasks that haven't been finished but have been revealed to agents;
    vector< deque<Task > > assigned_tasks;

    vector<list<std::tuple<int,int,std::string>>> events;
    list<Task> all_tasks;

    //for evaluation
    vector<int> solution_costs;
    int num_of_task_finish = 0;
    list<double> planner_times; 
    bool fast_mover_feasible = true;


    void initialize();
    bool planner_initialize();


    // deque<Task> task_queue;
    std::vector<int>& tasks;
    std::vector<int> task_counter;
    int tasks_size;
    int task_id = 0;
    virtual void update_tasks();
    virtual void sync_shared_env();

    list<Task> move(vector<Action>& actions);
    bool valid_moves(vector<State>& prev, vector<Action>& next);

    void log_preprocessing(bool succ);
    void log_event_assigned(int agent_id, int task_id, int timestep);
    void log_event_finished(int agent_id, int task_id, int timestep);

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



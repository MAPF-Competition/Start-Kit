#include "SharedEnv.h"
#include "Tasks.h"
#include "States.h"

#include "nlohmann/json.hpp"
#include <vector>
#include "Logger.h"

class TaskManager{
public:

    vector<list<std::tuple<int,int,std::string>>> events;
    vector<list<pair<int,int>>> actual_schedule;
    vector<list<pair<int,int>>> planner_schedule;
    list<std::tuple<std::string,int,int,int,int>> schedule_errors;

    list<int> check_finished_tasks(vector<State> states, int timestep);

    int curr_timestep;


    // reveal new task
    void reveal_tasks(int timestep);
    void update_tasks(vector<State> states, vector<int> assignment, int timestep);

    void sync_shared_env(SharedEnvironment* env);

    void set_num_tasks_reveal(int num){num_tasks_reveal = num*num_of_agents;};
    void set_logger(Logger* logger){this->logger = logger;}

    bool validate_task_assgnment(vector<int> assignment); // validate the task assignment
    bool set_task_assignment( vector<int>  assignment); // set the task assignment; return true if task is valid



    TaskManager(std::vector<list<int>>& tasks, int num_of_agents):
        tasks(tasks), num_of_agents(num_of_agents)
    {
        finished_tasks.resize(num_of_agents);
        current_assignment.resize(num_of_agents);
        for (auto & t: current_assignment)
            t = -1;
        events.resize(num_of_agents);
        actual_schedule.resize(num_of_agents);
        planner_schedule.resize(num_of_agents);
    }

    nlohmann::ordered_json to_json(int map_cols) const;

    nlohmann::ordered_json release_to_json() const;



    int num_of_task_finish = 0;

    ~ TaskManager()
    {
        for (Task* task: all_tasks){
            delete task;
        }
    }

private:
    Logger* logger = nullptr;

    unordered_map<int, Task*> ongoing_tasks;
    vector<int> current_assignment;

    int num_tasks_reveal = 1;
    int num_of_agents;

    std::vector<std::list<Task* > > finished_tasks; // location + finish time

    list<Task*> all_tasks;


    std::vector<list<int>>& tasks;
    int task_id = 0;

};

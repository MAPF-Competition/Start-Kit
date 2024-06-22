#include "SharedEnv.h"
#include "Tasks.h"
#include "States.h"

#include "nlohmann/json.hpp"

class TaskManager{
public:

    list<Task> check_finished_tasks(vector<State> states, int timestep);

    virtual bool update_tasks(int timestep);

    void sync_shared_env(SharedEnvironment* env);

    void set_num_tasks_reveal(int num){num_tasks_reveal = num;};

    TaskManager(std::vector<int>& tasks, int num_of_agents,
                vector<list<std::tuple<int,int,std::string>>>& events ):
        tasks(tasks), tasks_size(tasks.size()), num_of_agents(num_of_agents),
        events(events)
    {
        task_counter.resize(num_of_agents,0);
        finished_tasks.resize(num_of_agents);
        assigned_tasks.resize(num_of_agents);
    }

    nlohmann::ordered_json to_json(int map_cols) const;

    vector< deque<Task > > assigned_tasks;
    int num_of_task_finish = 0;

private:

    int num_tasks_reveal = 1;
    int num_of_agents;

    std::vector<std::list<Task > > finished_tasks; // location + finish time



    vector<list<std::tuple<int,int,std::string>>>& events;

    list<Task> all_tasks;


    std::vector<int>& tasks;
    std::vector<int> task_counter;
    int tasks_size;
    int task_id = 0;

};

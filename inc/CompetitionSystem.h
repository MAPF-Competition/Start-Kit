#pragma once
// #include "BasicSystem.h"
#include "SharedEnv.h"
#include "Grid.h"
#include "Tasks.h"
#include "ActionModel.h"
#include "MAPFPlanner.h"

class BaseSystem{
 public:


	BaseSystem(Grid &grid, MAPFPlanner* planner, ActionModelWithRotate* model):
    map(grid), planner(planner), env(planner->env), model(model)
  {}

	virtual ~BaseSystem(){};

	void simulate(int simulation_time);

  void savePaths(const string &fileName, int option) const; //option = 0: save actual movement, option = 1: save planner movement
  void saveErrors(const string &fileName) const;
  void saveResults(const string &fileName) const;

  int num_tasks_reveal = 1;

  void set_num_tasks_reveal(int num){num_tasks_reveal = num;};
  void set_plan_time_limit(int limit){plan_time_limit = limit;};



 protected:

  Grid map;

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

  vector<int> solution_costs;
  int num_of_task_finish = 0;

	void initialize();
	virtual void update_tasks() = 0;

  void sync_shared_env();


  vector<Action> plan();

  list<Task> move(vector<Action>& actions);
  bool valid_moves(vector<State>& prev, vector<Action>& next);
};


class FixedAssignSystem : public BaseSystem

{
 public:
	FixedAssignSystem(Grid &grid, string agent_task_filename, MAPFPlanner* planner, ActionModelWithRotate *model):
    BaseSystem(grid, planner, model)
  {
    load_agent_tasks(agent_task_filename);
  };

	FixedAssignSystem(Grid &grid, MAPFPlanner* planner, std::vector<int>& start_locs, std::vector<vector<int>>& tasks, ActionModelWithRotate* model):
    BaseSystem(grid, planner, model)
  {
    if (start_locs.size() != tasks.size()){
      std::cerr << "agent num does not match the task assignment" << std::endl;
      exit(1);
    }

    int task_id = 0;
    num_of_agents = start_locs.size();
    starts.resize(num_of_agents);
    task_queue.resize(num_of_agents);
    for (size_t i = 0; i < start_locs.size(); i++){
      starts[i] = State(start_locs[i], 0, 0);
      for (auto& task_location: tasks[i]){
        all_tasks.emplace_back(task_id++, task_location, 0, (int)i);
        task_queue[i].emplace_back(all_tasks.back().task_id, all_tasks.back().location, all_tasks.back().t_assigned, all_tasks.back().agent_assigned);
      }
      // task_queue[i] = deque<int>(tasks[i].begin(), tasks[i].end());
    }
  };

	~FixedAssignSystem(){};

  bool load_agent_tasks(string fname);


 private:
  vector<deque<Task>> task_queue;

	void update_tasks();

};


class TaskAssignSystem : public BaseSystem
{
public:
	TaskAssignSystem(Grid &grid, MAPFPlanner* planner, std::vector<int>& start_locs, std::vector<int>& tasks, ActionModelWithRotate* model):
    BaseSystem(grid, planner, model)
  {
    int task_id = 0;
    for (auto& task_location: tasks){
      all_tasks.emplace_back(task_id++, task_location);
      task_queue.emplace_back(all_tasks.back().task_id, all_tasks.back().location);
      //task_queue.emplace_back(task_id++, task_location);
    }
    num_of_agents = start_locs.size();
    starts.resize(num_of_agents);
    for (size_t i = 0; i < start_locs.size(); i++){
      starts[i] = State(start_locs[i], 0, 0);
    }
  };


	~TaskAssignSystem(){};

private:
  deque<Task> task_queue;

	void update_tasks();

};

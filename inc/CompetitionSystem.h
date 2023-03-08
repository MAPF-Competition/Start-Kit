#pragma once
// #include "BasicSystem.h"
#include "SharedEnv.h"
#include "Grid.h"
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

  int num_tasks_reveal = 1;

  void set_num_tasks_reveal(int num){num_tasks_reveal = num;};


 protected:

  Grid map;

  MAPFPlanner* planner;
  SharedEnvironment* env;

  ActionModelWithRotate* model;

  // #timesteps for simulation
  int timestep;

  int preprocess_time_limit;
  int plan_time_limit;

  std::vector<Path> paths;
  std::vector<std::list<std::pair<int, int> > > finished_tasks; // location + finish time

  vector<State> starts;
  int num_of_agents;

  vector<State> curr_states;

  vector<list<Action>> actual_movements;
  vector<list<Action>> planner_movements;

  // tasks that haven't been finished but have been revealed to agents;
  vector< vector<pair<int, int> > > goal_locations;

	void initialize();
	virtual void update_goal_locations() = 0;

  void sync_shared_env();

  // move agents,  update agents location, return finished tasks
  list<tuple<int, int, int>> move(vector<Action>& actions);
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

    num_of_agents = start_locs.size();
    starts.resize(num_of_agents);
    task_queue.resize(num_of_agents);
    for (size_t i = 0; i < start_locs.size(); i++){
      starts[i] = State(start_locs[i], 0, 0);
      task_queue[i] = deque<int>(tasks[i].begin(), tasks[i].end());
    }
  };

	~FixedAssignSystem(){};

  bool load_agent_tasks(string fname);


 private:
  vector<deque<int>> task_queue;

	void update_goal_locations();

};


class TaskAssignSystem : public BaseSystem
{
public:
	TaskAssignSystem(Grid &grid, MAPFPlanner* planner, std::vector<int>& start_locs, std::vector<int>& tasks, ActionModelWithRotate* model):
    BaseSystem(grid, planner, model), task_queue(tasks.begin(), tasks.end())
  {
    num_of_agents = start_locs.size();
    starts.resize(num_of_agents);
    for (size_t i = 0; i < start_locs.size(); i++){
      starts[i] = State(start_locs[i], 0, 0);
    }
  };


	~TaskAssignSystem(){};

private:
  deque<int> task_queue;

	void update_goal_locations();

};

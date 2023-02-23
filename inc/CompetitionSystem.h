#pragma once
// #include "BasicSystem.h"
#include "SharedEnv.h"
#include "Grid.h"
#include "Validator.h"
#include "MAPFPlanner.h"

class BaseSystem{
 public:


	BaseSystem(Grid &grid, MAPFPlanner* planner, Validator* validator=nullptr):
    map(grid), planner(planner), env(planner->env), validator(validator)
  {}

	virtual ~BaseSystem(){};

	void simulate(int simulation_time);

  void savePaths(const string &fileName, int option) const; //option = 0: save actual movement, option = 1: save planner movement
  void saveErrors(const string &fileName) const;

 protected:

  Grid map;

  MAPFPlanner* planner;
  SharedEnvironment* env;

  Validator* validator;

  // #timesteps for simulation
  int timestep;

  int preprocess_time_limit;
  int plan_time_limit;

  std::vector<Path> paths;
  std::vector<std::list<std::pair<int, int> > > finished_tasks; // location + finish time

  vector<State> starts;
  int num_of_agents;

  vector<State> curr_states;

  vector<list<State>> actual_movements;
  vector<list<State>> planner_movements;

  // tasks that haven't been finished but have been revealed to agents;
  vector< vector<pair<int, int> > > goal_locations;

	void initialize();
	virtual void update_goal_locations() = 0;

  void sync_shared_env();

  // move agents,  update agents location, return finished tasks
  list<tuple<int, int, int>> move(vector<State>& next_states);
  bool valid_moves(vector<State>& prev, vector<State>& next);
};


class FixedAssignSystem : public BaseSystem

{
 public:
	FixedAssignSystem(Grid &grid, string agent_task_filename, MAPFPlanner* planner, Validator* validator=nullptr):
    BaseSystem(grid, planner, validator)
  {
    load_agent_tasks(agent_task_filename);
  };

	FixedAssignSystem(Grid &grid, MAPFPlanner* planner, std::vector<int>& start_locs, std::vector<vector<int>>& tasks, Validator* validator=nullptr):
    BaseSystem(grid, planner, validator)
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
	TaskAssignSystem(Grid &grid, MAPFPlanner* planner, std::vector<int>& start_locs, std::vector<int>& tasks, Validator* validator=nullptr):
    BaseSystem(grid, planner, validator), task_queue(tasks.begin(), tasks.end())
  {
    num_of_agents = start_locs.size();
    starts.resize(num_of_agents);
    for (size_t i = 0; i < start_locs.size(); i++){
      starts[i] = State(start_locs[i], 0, 0);
    }
  };

  int num_tasks_reveal = 1;
  string assign_strategy = "next-agent-next-task";

  void set_num_tasks_reveal(int num){num_tasks_reveal = num;};
  void set_assign_strategy(string strategy){assign_strategy = strategy;};

	~TaskAssignSystem(){};

private:
  deque<int> task_queue;

	void update_goal_locations();

};

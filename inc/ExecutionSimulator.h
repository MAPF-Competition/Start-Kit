#pragma once
#include "ActionModel.h"
#include "FreeState.h"
#include "Grid.h"
#include "Logger.h"
#include "States.h"
#include <boost/asio/connect.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/address.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/core/tcp_stream.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/http/message.hpp>
#include <boost/beast/http/string_body.hpp>
#include <boost/beast/http/verb.hpp>
#include <boost/beast/version.hpp>
#include <string>
#include <vector>
#include <random>
// You are responsible telling the robot's central controller the plan
// And getting the position of robots from the central controller
// The central controller will decide the execution policy to implement the plan
// Maybe you should maintain a latest copy of the Controller's world view to
// reduce latency?

// You might be implemented as a HTTP server?
// Where should plans be stored, who needs to know what has been completed so
// far?

// For now, nobody 'needs' to know what we've done so far. Just execute without
// feedback

// The executor of planned and validated actions, takes a vector of states and
// sends this to the central controller for the agents which runs an execution
// policy

class ActionExecutor {
public:
  // ActionExecutor(Grid & planner_grid, Grid & real_grid):
  // planner_grid(planner_grid), rows(planner_grid.rows),
  // cols(planner_grid.cols), real_grid(real_grid){}
  ActionExecutor(){};
  void send_plan(vector<State> &curr, vector<State> &next);
  vector<State> get_agent_locations(int timestep);
  vector<bool> get_agent_success(int timestep);
  void set_logger(Logger *logger) { this->logger = logger; }


  // virtual ~ActionExecutor();
  // Communication with ExecutionPolicy

  // Expects the actual_movements are prepared such that all agents have an action for all timesteps stored
  // int simulate_batch(vector<vector<Action>> &movements, ActionModelWithRotate* model, int max_timesteps) {
  //   vector<State> curr_states = this->get_agent_locations(0);
  //   std::cout << "Beginning simulation" << std::endl;
  //   std::cout << std::to_string(max_timesteps) << " timesteps for " << std::to_string(movements.size()) << " agents" << std::endl;
  //   vector<Action> next_actions(movements.size());

  //   for (int timestep = 0; timestep < max_timesteps; timestep++) {
  //       for (int j = 0; j < movements.size(); j++) {
  //         std::cout << "Agent " << std::to_string(j) << " has " << std::to_string(movements.at(j).size()) << " actions" << std::endl;
  //           std::cout << "Agent " << std::to_string(j) << " moves at time " << std::to_string(timestep) << std::endl;
  //           next_actions.at(timestep) = movements.at(j)[timestep];
  //         std::cout << "Agent " << std::to_string(j) << " succeeds" << std::endl;
  //       }

  //       vector<State> next_states = model->result_states(curr_states, next_actions);
  //       this->send_plan(curr_states, next_states);

  //       curr_states = this->get_agent_locations(timestep);
  //      next_actions.clear();
  //   }
  // }

protected:
  Logger *logger = nullptr;
};

class PerfectExecutor : public ActionExecutor {
public:
  PerfectExecutor() : ActionExecutor(){};
  // curr_states and next_States under predicted next step if perfectly executed
  void send_plan(vector<State> &curr, vector<State> &next) { next_states = next; }

  vector<State> get_agent_locations(int timestep) {
    return next_states;
  }

  ~PerfectExecutor(){};

private:
  vector<State> next_states;
};

class TurtlebotExecutor : public ActionExecutor {
public:
  TurtlebotExecutor(int rows, int cols)
      : rows_(rows), cols_(cols), 
        ActionExecutor(){};
  // Setup http connection as websocket?
  vector<State> get_agent_locations(int timestep);
  void send_plan(vector<State> &curr,vector<State> &next);

  ~TurtlebotExecutor(){};

private:
  int rows_;
  int cols_;

  const string hostname = "192.168.0.141";
  const string port =
      "8080"; // hsotname and port the central controller has a TCP listener on
  boost::asio::io_service ioc_;

  inline float location_to_x(int location) {
    return static_cast<float>(location % cols_);
  };

  inline float location_to_y(int location) {
    return static_cast<float>(location / cols_);
  };

  inline int xy_to_location(int x, int y) { return y * cols_ + x; }

  FreeState transform_state(State &place) {
    std::cout << place.location << " " << location_to_x(place.location) << " "
              << location_to_y(place.location) << " " << cols_ << std::endl;
    return FreeState{.x = location_to_x(place.location),
                     .y = location_to_y(place.location),
                     .theta = static_cast<float>(place.orientation * 90),
                     .timestep = place.timestep};
  }

  vector<FreeState> prepare_next_agent_poses(vector<State> &next) {
    std::cout << "Size of next poses: " << std::to_string(next.size()) << std::endl;
    vector<FreeState> next_agent_poses(next.size());
    for (int i = 0; i < next.size(); i++) {
      next_agent_poses[i] = transform_state(next[i]);
    }
    return next_agent_poses;
  };
};

class ProbabilisticExecutor : ActionExecutor 
{
public:
  ProbabilisticExecutor(float success_chance) :
  success_chance_(success_chance), successes_(vector<bool>()),states_(vector<State>()), rd(), gen(rd()), distrib(0, 1),
   ActionExecutor(){}

  void send_plan(vector<State> &curr, vector<State> &next) 
  {
    successes_.resize(curr.size());
    for (bool success : successes_) {
      success = (bool)distrib(gen);    
    }

    states_.resize(curr.size());
    for (int i = 0; i < successes_.size(); i++) {
      states_[i] =  successes_[i] ? next[i] : curr[i];
    }
  }

  vector<bool> get_agent_success(int timestep) {
      return successes_;
  }
  
  vector<State> get_agent_locations(int timestep) {
    return states_;
  }
  
private:
  float success_chance_;
  vector<bool> successes_;  
  vector<State> states_;
  std::random_device rd;
  std::mt19937 gen;
  std::uniform_int_distribution<int> distrib;
};

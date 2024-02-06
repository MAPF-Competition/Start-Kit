#pragma once
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
namespace beast = boost::beast; // from <boost/beast.hpp>
namespace http = beast::http;   // from <boost/beast/http.hpp>
namespace net = boost::asio;    // from <boost/asio.hpp>

class ActionExecutor {
public:
  // ActionExecutor(Grid & planner_grid, Grid & real_grid):
  // planner_grid(planner_grid), rows(planner_grid.rows),
  // cols(planner_grid.cols), real_grid(real_grid){}
  ActionExecutor(){};
  virtual void send_plan(vector<State> &next){};
  virtual vector<State> get_agent_locations(int timestep){};
  void set_logger(Logger *logger) { this->logger = logger; }

  // Transforms between planner's map and execution map
  State real_to_planner(FreeState &);
  FreeState place_on_map(State &place);

  // virtual ~ActionExecutor();
  // Communication with ExecutionPolicy

protected:
  Logger *logger = nullptr;
};

class PerfectExecutor : public ActionExecutor {
public:
  PerfectExecutor() : ActionExecutor(){};

  void send_plan(vector<State> &next) override { next_states = next; }

  vector<State> get_agent_locations(int timestep) override {
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
  virtual vector<State> get_agent_locations(int timestep) override;
  virtual void send_plan(vector<State> &next) override;

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
    vector<FreeState> next_agent_poses(next.size());
    for (int i = 0; i < next.size(); i++) {
      next_agent_poses[i] = transform_state(next[i]);
    }
    return next_agent_poses;
  };
};
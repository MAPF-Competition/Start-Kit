#pragma once
#include <boost/asio/basic_socket.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/detail/endpoint.hpp>
#include <boost/system/error_code.hpp>
#include <string>
#include <vector>
#include "Grid.h"
#include "States.h"
#include "Logger.h"
#include "FreeState.h"
#include <boost/asio.hpp>
#include <boost/asio/ip/address.hpp>
// You are responsible telling the robot's central controller the plan
// And getting the position of robots from the central controller
// The central controller will decide the execution policy to implement the plan
// Maybe you should maintain a latest copy of the Controller's world view to reduce latency?

// You might be implemented as a HTTP server?
// Where should plans be stored, who needs to know what has been completed so far?

// For now, nobody 'needs' to know what we've done so far. Just execute without feedback

// The executor of planned and validated actions, takes a vector of states and
// sends this to the central controller for the agents which runs an execution policy
using namespace boost::asio;
class ActionExecutor
{
public:
    // ActionExecutor(Grid & planner_grid, Grid & real_grid): planner_grid(planner_grid), rows(planner_grid.rows), cols(planner_grid.cols), real_grid(real_grid){}
    ActionExecutor(){};
    virtual void send_plan(vector<State>& next) {};
    virtual vector<State> get_agent_locations(int timestep) {};
    void set_logger(Logger* logger){this->logger = logger;}

    // Transforms between planner's map and execution map
    State real_to_planner (FreeState&);
    FreeState place_on_map(State& place);
    
    // virtual ~ActionExecutor();
    // Communication with ExecutionPolicy


protected:
    Logger* logger = nullptr;

};

class PerfectExecutor : public ActionExecutor
{
public:
    PerfectExecutor():
        ActionExecutor()
    {};

    
    void send_plan(vector<State>& next) override 
    {
        next_states = next;
    }

    vector<State> get_agent_locations(int timestep) override 
    {
        return next_states;
    }

    ~PerfectExecutor(){};

private:
    vector<State> next_states;
};

// Do a concrete implementation for turtlebots, with rotate, delay probabilities, wiggly movement
// Localising independently...
// It feels like the plan sending and state retrieval should be polymorphic... but is it too much abstraction
// I think anyone sane should only implement 1-2 ways to send plans and 1-2 ways to get states so no abstraction seems okay

class TurtlebotExecutor : public ActionExecutor
{
    
public:
    TurtlebotExecutor(int rows, int cols):
        rows(rows), cols(cols), ActionExecutor(){};
        // Setup http connection as websocket?
    virtual vector<State> get_agent_locations(int timestep) override;
    virtual void send_plan(vector<State>& next) override;
private:
    int rows;
    int cols;

    const ip::tcp::endpoint controller_endpoint = {ip::address::from_string("192.168.0.141"), 8080}; // Address of turtlebot controller server in LAN

    inline float location_to_x(int location) {
        return static_cast<float>(location % cols);
    };

    inline float location_to_y(int location) {
        return static_cast<float>(location / cols);
    };

    inline int xy_to_location(int x, int y) {
        return y * cols + x;
    }

    FreeState transform_state(State& place)
    {
        return FreeState{.x = location_to_x(place.location), .y = location_to_y(place.location), .theta= static_cast<float>(place.orientation*90), .timestep= place.timestep};
    }

    vector<FreeState> prepare_next_agent_poses(vector<State>& next) 
    {
        vector<FreeState> next_agent_poses(next.size());
        for (int i = 0; i < next.size(); i++) 
        {
            next_agent_poses[i] = transform_state(next[i]);
        }
        return next_agent_poses;
    };

};
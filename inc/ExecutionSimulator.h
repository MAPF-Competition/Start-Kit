#pragma once
#include <string>
#include <vector>
#include "Grid.h"
#include "States.h"
#include "Logger.h"
#include "FreeState.h"

// You are responsible telling the robot's central controller the plan
// And getting the position of robots from the central controller
// The central controller will decide the execution policy to implement the plan
// Maybe you should maintain a latest copy of the Controller's world view to reduce latency?

// You might be implemented as a HTTP server?
// Where should plans be stored, who needs to know what has been completed so far?

// For now, nobody 'needs' to know what we've done so far. Just execute without feedback

// The executor of planned and validated actions, takes a vector of states and
// sends this to the central controller for the agents which runs an execution policy
class ActionExecutor
{
public:
    // ActionExecutor(Grid & planner_grid, Grid & real_grid): planner_grid(planner_grid), rows(planner_grid.rows), cols(planner_grid.cols), real_grid(real_grid){}
    ActionExecutor(){};
    virtual void send_plan(const vector<State>& next) {};
    virtual vector<State> get_agent_locations(int timestep) {};
    void set_logger(Logger* logger){this->logger = logger;}

    // Transforms between planner's map and execution map
    State real_to_planner (FreeState&);
    FreeState place_on_map(State& place);
    
    // virtual ~ActionExecutor();
    // Communication with ExecutionPolicy


protected:
    Logger* logger = nullptr;
    
    int rows;
    int cols;

    
    float location_to_x(int location) {
        return static_cast<float>(location % rows);
    };

    float location_to_y(int location) {
        return static_cast<float>(location / rows);
    };

    // TODO: Test this conversion
    float orientation_to_theta(int orientation) {
        switch (orientation) {
            case 0:
                return 0.0; break;
            case 1:
                return 270.0; break;
            case 2:
                return 180.0; break;
            case 3: 
                return 90.0; break;
            default:
                // TODO: Log something for invalid orientation
                return 42.0; break; // Magic error number or just crash at runtime?
        }
    };

    

    vector<FreeState> prepare_next_agent_poses(vector<State>& next) {
        vector<FreeState> next_agent_poses(next.size());
        for (int i = 0; i < next.size(); i++) {
            next_agent_poses[i] = place_on_map(next[i]);
        }
        return next_agent_poses;
    };
};

class PerfectExecutor : public ActionExecutor
{
public:
    PerfectExecutor():
        ActionExecutor()
    {};

    
    void send_plan(const vector<State>& next) override {
        next_states = next;
    }

    vector<State> get_agent_locations(int timestep) override {
        return next_states;
    }

    ~PerfectExecutor(){};

private:
    vector<State> next_states;
};
#pragma once
#include <string>
#include "Grid.h"
#include "States.h"
#include "Logger.h"

/*
  FW  - forward
  CR  - Clockwise rotate
  CCR - Counter clockwise rotate
  W   - Wait
  NA  - Not applicable
*/
enum Action {FW, CR, CCR, W, NA};

std::ostream& operator<<(std::ostream &stream, const Action &action);

class ActionModelWithRotate
{
public:
    list<std::tuple<std::string,int,int,int>> errors;

    ActionModelWithRotate(Grid & grid): grid(grid), rows(grid.rows), cols(grid.cols){
        moves[0] = 1;
        moves[1] = cols;
        moves[2] = -1;
        moves[3] = -cols;

    };

    bool is_valid(vector<State>& prev, const vector<Action> & action, int timestep);
    void set_logger(Logger* logger){this->logger = logger;}

    vector<State> result_states(vector<State>& prev, const vector<Action> & action){
        vector<State> next(prev.size());
        for (size_t i = 0 ; i < prev.size(); i ++){
            next[i] = result_state(prev[i], action[i]);
        }
        return next;
    };


protected:
    const Grid& grid;
    int rows;
    int cols;
    int moves[4];
    Logger* logger = nullptr;

    State result_state(State & prev, Action action)
    {
        if (prev.delay.inDelay())
        {
            // If in delay, cannot perform any action other than wait
            prev.delay.tick();
            return State(prev.location, prev.timestep + 1, prev.orientation, prev.counter, prev.delay);
        }
        int new_location = prev.location;
        int new_orientation = prev.orientation;
        if (action == Action::FW)
        {
            if(prev.counter.tick())
                new_location = new_location += moves[prev.orientation];
        }
        else if (action == Action::CR)
        {
            if(prev.counter.tick())
                new_orientation = (prev.orientation + 1) % 4;
      
        }
        else if (action == Action::CCR)
        {
            if(prev.counter.tick())
            {
                new_orientation = (prev.orientation - 1) % 4;
                if (new_orientation == -1)
                    new_orientation = 3;
            }
        }

        return State(new_location, prev.timestep + 1, new_orientation, prev.counter, prev.delay);
    }
};

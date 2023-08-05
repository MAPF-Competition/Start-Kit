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

    bool is_valid(const vector<State>& prev, const vector<Action> & action);
    void set_logger(Logger* logger){this->logger = logger;}

    vector<State> result_states(const vector<State>& prev, const vector<Action> & action){
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

    State result_state(const State & prev, Action action)
    {
        int new_location = prev.location;
        int new_orientation = prev.orientation;
        if (action == Action::FW)
        {
            new_location = new_location += moves[prev.orientation];
        }
        else if (action == Action::CR)
        {
            new_orientation = (prev.orientation + 1) % 4;
      
        }
        else if (action == Action::CCR)
        {
            new_orientation = (prev.orientation - 1) % 4;
            if (new_orientation == -1)
                new_orientation = 3;
        }

        return State(new_location, prev.timestep + 1, new_orientation);
    }
};

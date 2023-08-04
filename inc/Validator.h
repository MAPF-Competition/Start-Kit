#pragma once
#include <string>
#include "Grid.h"
#include "States.h"

class Validator
{
protected:
    std::string msg;
public:
    std::string error_msg()
    {
        return msg;
    }
    virtual bool is_valid(vector<State>& prev, vector<State> & next)=0;

    virtual ~Validator(){};
    list<std::tuple<std::string,int,int,int>> errors;
};


class ValidatorRotate : public Validator
{
protected:
    const Grid& grid;
    int rows;
    int cols;

    int moves[4];

public:
    ValidatorRotate(Grid & grid): grid(grid), rows(grid.rows), cols(grid.cols)
    {
        moves[0] = 1;
        moves[1] = -cols;
        moves[2] = -1;
        moves[3] = cols;

    };

    virtual bool is_valid(vector<State>& prev, vector<State> & next) override;

    //ERRORs, 
    // <ERROR CODE, Agent 1 (-1 if N/A), Agent 2 (-1 if N/A), Timestep>
    // ERROR CODE:
    // 0: over-rotates
    // 1: moves and rotates at the same time
    // 2: moves in a wrong direction
    // 3: moves more than 1 steps
    // 4: moves to an obstacle
    // 5: vertex conflict
    // 6: edge conflict
    // 7: missing plans (size of the agents plans does not match the number of agents)
};

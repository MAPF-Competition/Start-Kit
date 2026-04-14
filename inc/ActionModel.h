#pragma once
#include <string>
#include "Grid.h"
#include "States.h"
#include "Logger.h"

std::ostream& operator<<(std::ostream &stream, const Action &action);

class ActionModelWithRotate
{
public:
    list<std::tuple<std::string,int,int,int>> errors;

    ActionModelWithRotate(Grid & grid, float agent_size):
        grid(grid), rows(grid.rows), cols(grid.cols), _agent_size(agent_size){
        moves[0] = 1;
        moves[1] = cols;
        moves[2] = -1;
        moves[3] = -cols;

    };

    vector<State> step(const vector<State>& prev, vector<Action> & action, int timestep);
    void set_logger(Logger* logger){this->logger = logger;}

    vector<State> result_states(const vector<State>& prev, const vector<Action> & action);
    list<std::tuple<std::string,int,int,int>> get_errors(){ return errors; }
    vector<char> get_wait_agents(){ return _wait_agents; }

protected:
    const Grid& grid;
    int rows;
    int cols;
    int moves[4];
    Logger* logger = nullptr;

    State result_state(const State & prev, Action action);

    struct RealLocation
    {
        float x;
        float y;
    };
    vector<RealLocation> get_real_locations(const vector<State>& state); 
    void sanity_check_states(const vector<State>& states);

private:
    float _agent_size;
    float _overlap_eps = 1e-4f;
    vector<char> _wait_agents;
};

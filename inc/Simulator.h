#include "ActionModel.h"
#include "Tasks.h"


//simulator is use for simulating the moves
//has function (1) simulate move (based on the given actions and current states)
//             (2) valiation (valiate if the given action is valid or not)
// question: do we store current states and curr tasks in simulator or in competition system?
// where we store current task allocations?

class Simulator
{
public:
    Simulator(Grid &grid, ActionModelWithRotate* model):
        map(grid), model(model)
    {}

    list<Task> move(vector<Action>& next_actions, vector<Task>& proposed_toptasks, const vector<State>& curr_states);

private:
    Grid map;

    ActionModelWithRotate* model;

    bool is_valid(const vector<State>& prev, const vector<Action> & action);
    
}
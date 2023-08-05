#include "ActionModel.h"


std::ostream& operator<<(std::ostream &stream, const Action &action)
{
    if (action == Action::FW) {
        stream << "F";
    } else if (action == Action::CR) {
        stream << "R";
    } else if (action == Action::CCR) {
        stream << "C";
    } else {
        stream << "W";
    }

    return stream;
}


bool ActionModelWithRotate::is_valid(const vector<State>& prev, const vector<Action> & actions)
{
    if (prev.size() != actions.size())
    {
        errors.push_back(make_tuple("incorrect vector size",-1,-1,prev[0].timestep+1));
        return false;
    }

    vector<State> next = result_states(prev, actions);
    unordered_map<int, int> vertex_occupied;
    unordered_map<pair<int, int>, int> edge_occupied;

    for (int i = 0; i < prev.size(); i ++) 
    {

        if (grid.map[next[i].location] == 1)
        {
            cout << "ERROR: agent " << i << " moves to an obstacle. " << endl;
            errors.push_back(make_tuple("unallowed move",i,-1,next[i].timestep));
            return false;
        }


        if (vertex_occupied.find(next[i].location) != vertex_occupied.end()) {
            cout << "ERROR: agents " << i << " and " << vertex_occupied[next[i].location] << " have a vertex conflict. " << endl;
            errors.push_back(make_tuple("vertex conflict",i,vertex_occupied[next[i].location], next[i].timestep));
            return false;
        }

        int edge_idx = (prev[i].location + 1) * rows * cols +  next[i].location;

        if (edge_occupied.find({prev[i].location, next[i].location}) != edge_occupied.end()) {
            cout << "ERROR: agents " << i << " and " << edge_occupied[{prev[i].location, next[i].location}] << " have an edge conflict. " << endl;
            errors.push_back(make_tuple("edge conflict", i, edge_occupied[{prev[i].location, next[i].location}], next[i].timestep));
            return false;
        }
        

        vertex_occupied[next[i].location] = i;
        int r_edge_idx = (next[i].location + 1) * rows * cols +  prev[i].location;
        edge_occupied[{next[i].location, prev[i].location}] = i;
    }

    return true;
}

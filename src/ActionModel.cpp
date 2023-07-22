#include "ActionModel.h"

std::ostream& operator<<(std::ostream &stream, const Action &action){
    if (action == Action::FW){
        stream << "F";
    } else if (action == Action::CR){
        stream << "R";
    } else if (action == Action::CCR){
        stream << "C";
    } else {
        stream << "W";
    }

    return stream;
}

bool ActionModelWithRotate::is_valid(const vector<State>& prev, const vector<Action> & actions){
    if (prev.size() != actions.size()) {
        errors.push_back(make_tuple("incorrect vector size",-1,-1,prev[0].timestep+1));
        return false;
    }

    vector<State> next = result_states(prev, actions);
    unordered_map<int, int> vertex_occupied;
    unordered_map<pair<int, int>, int> edge_occupied;

    for (int i = 0; i < prev.size(); i ++) {
        /*
          if (prev[i].location == next[i].location) {
          // check if the rotation is not larger than 90 degree
          if (abs(prev[i].orientation - next[i].orientation) == 2){
          cout << "ERROR: agent " << i << " over-rotates. " << endl;
          errors.push_back(make_tuple("over-rotate",i,-1,next[i].timestep));
          return false;
          }
          } else {
          if (prev[i].orientation != next[i].orientation){
          cout << "ERROR: agent " << i << " moves and rotates at the same time. " << endl;
          errors.push_back(make_tuple("unallowed move",i,-1,next[i].timestep));
          return false;
          }
          if (next[i].location - prev[i].location != moves[prev[i].orientation]){
          cout << "ERROR: agent " << i << " moves in a wrong direction. " << endl;
          errors.push_back(make_tuple("unallowed move",i,-1,next[i].timestep));
          return false;
          }

          if (abs(next[i].location / cols - prev[i].location/cols) + abs(next[i].location % cols - prev[i].location %cols) > 1  ){
          cout << "ERROR: agent " << i << " moves more than 1 steps. " << endl;
          errors.push_back(make_tuple("unallowed move",i,-1,next[i].timestep));
          return false;
          }
          }
        */
        if (grid.map[next[i].location] == 1) {
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
        
        //orthogonal movement collisions
        //check if next.location to other location, this happen only when agents are doing forwarding
        // if (prev[i].location != next[i].location)
        // {
        //     int loc1 = next[i].location;
        //     int candidates[4] = { loc1 + 1,loc1 + cols, loc1 - 1, loc1 - cols};
        //     for (auto loc2: candidates)
        //     {
        //         if (loc2 == prev[i].location)
        //             continue;
        //         if ((loc2 - loc1) == (loc1 - prev[i].location))
        //             continue;
        //         edge_idx = (loc2+1)* rows * cols + loc1;

        //         if (occupied.find(edge_idx) != occupied.end()) {
        //         cout << "ERROR: agents " << i << " and " << occupied[edge_idx] << " have an vertex conflict. " << endl;
        //         errors.push_back(make_tuple("vertex conflict",i,occupied[edge_idx],next[i].timestep));
        //         return false;
        //     }
        //     }
        // }

        vertex_occupied[next[i].location] = i;
        int r_edge_idx = (next[i].location + 1) * rows * cols +  prev[i].location;
        edge_occupied[{next[i].location, prev[i].location}] = i;
    }

    return true;
}

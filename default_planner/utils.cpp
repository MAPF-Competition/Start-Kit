


#include "utils.h"

namespace DefaultPlanner{
int get_d(int diff, const SharedEnvironment* env)  {

    return (diff == 1)? 0: (diff == -1)? 2: (diff == env->cols)? 1: 3;

}

bool validateMove(int loc, int loc2, const SharedEnvironment* env)
{
    int loc_x = loc/env->cols;
    int loc_y = loc%env->cols;

    if (loc_x < 0 || loc_y < 0 || loc_x >= env->rows || loc_y >= env->cols || env->map[loc] == 1)
        return false;

    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;
	 if (loc2_x < 0 || loc2_y < 0 ||loc2_x >= env->rows || loc2_y >= env->cols || env->map[loc2] == 1)
        return false;
    if (abs(loc_x-loc2_x) + abs(loc_y-loc2_y) > 1)
        return false;
    return true;

}

int manhattanDistance(int loc, int loc2,const SharedEnvironment* env){
	int loc_x = loc/env->cols;
	int loc_y = loc%env->cols;
	int loc2_x = loc2/env->cols;
	int loc2_y = loc2%env->cols;
	return abs(loc_x-loc2_x) + abs(loc_y-loc2_y);

}
void getNeighbors(const SharedEnvironment* env, std::vector<std::pair<int,int>>& neighbors, int location,int direction) {
    neighbors.clear();
	//forward
	assert(location >= 0 && location < env->map.size());
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
    int forward = candidates[direction];
    int new_direction = direction;
	assert(forward!=location);

	#ifndef NDEBUG
			std::cout<<"forward: "<<forward<<std::endl;
	#endif
    if (validateMove(location, forward, env)	){
		#ifndef NDEBUG
			std::cout<<"forward yes"<<std::endl;
		#endif
        neighbors.emplace_back(std::make_pair(forward,new_direction));
	}
    //turn left
    new_direction = direction-1;
    if (new_direction == -1)
        new_direction = 3;
	assert(new_direction >= 0 && new_direction < 4);
    neighbors.emplace_back(std::make_pair(location,new_direction));
    //turn right
    new_direction = direction+1;
    if (new_direction == 4)
        new_direction = 0;
	assert(new_direction >= 0 && new_direction < 4);
    neighbors.emplace_back(std::make_pair(location,new_direction));
    neighbors.emplace_back(std::make_pair(location,direction)); //wait
}

void getNeighbors_nowait(const SharedEnvironment* env, std::vector<std::pair<int,int>>& neighbors, int location,int direction) {
    neighbors.clear();
	//forward
	assert(location >= 0 && location < env->map.size());
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
    int forward = candidates[direction];
    int new_direction = direction;
    if (validateMove(location, forward, env)){
		assert(forward >= 0 && forward < env->map.size());
        neighbors.emplace_back(std::make_pair(forward,new_direction));
	}
    //turn left
    new_direction = direction-1;
    if (new_direction == -1)
        new_direction = 3;
	assert(new_direction >= 0 && new_direction < 4);
    neighbors.emplace_back(std::make_pair(location,new_direction));
    //turn right
    new_direction = direction+1;
    if (new_direction == 4)
        new_direction = 0;
	assert(new_direction >= 0 && new_direction < 4);
    neighbors.emplace_back(std::make_pair(location,new_direction));
}

void getNeighborLocs(const Neighbors* ns, std::vector<int>& neighbors, int location) {
    neighbors.clear();
	//forward
	assert(location >= 0 && location < ns->size());
    neighbors = ns->at(location);
    return;

}

void getNeighborLocs(const Neighbors* ns, int neighbors[], int location) {
	//forward
    int size = 4;
	assert(location >= 0 && location < ns->size());

	for (int i = 0; i < size; i++) {
		if (i < ns->at(location).size()){
			neighbors[i] = ns->at(location)[i];
		}
        else
            neighbors[i] = -1;
	}

}
}


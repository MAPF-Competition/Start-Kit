#ifndef utils_hpp
#define utils_hpp



template<typename T, typename F>
void quickSort(T& agent_order, int low, int high, F compare)
{
    if (low >= high)
        return;
    auto pivot = agent_order[high];    // pivot
    int i = low;  // Index of smaller element
    for (int j = low; j <= high - 1; j++)
    {
        // If true, current element is smaller than or equal to pivot
        // If true, a have higher priority
        if (compare(agent_order[j], pivot))
        {
            std::swap(agent_order[i], agent_order[j]);
            i++;    // increment index of smaller element
        }
    }
    std::swap(agent_order[i], agent_order[high]);

    quickSort(agent_order, low, i - 1, compare);  // Before i
    quickSort(agent_order, i + 1, high, compare); // After i
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
        neighbors.emplace_back(make_pair(forward,new_direction));
	}
    //turn left
    new_direction = direction-1;
    if (new_direction == -1)
        new_direction = 3;
	assert(new_direction >= 0 && new_direction < 4);
    neighbors.emplace_back(make_pair(location,new_direction));
    //turn right
    new_direction = direction+1;
    if (new_direction == 4)
        new_direction = 0;
	assert(new_direction >= 0 && new_direction < 4);
    neighbors.emplace_back(make_pair(location,new_direction));
    neighbors.emplace_back(make_pair(location,direction)); //wait
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
        neighbors.emplace_back(make_pair(forward,new_direction));
	}
    //turn left
    new_direction = direction-1;
    if (new_direction == -1)
        new_direction = 3;
	assert(new_direction >= 0 && new_direction < 4);
    neighbors.emplace_back(make_pair(location,new_direction));
    //turn right
    new_direction = direction+1;
    if (new_direction == 4)
        new_direction = 0;
	assert(new_direction >= 0 && new_direction < 4);
    neighbors.emplace_back(make_pair(location,new_direction));
}

void getNeighborLocs(const SharedEnvironment* env, std::vector<int>& neighbors, int location) {
    neighbors.clear();
	//forward
	assert(location >= 0 && location < env->map.size());
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};

	for (int i = 0; i < 4; i++) {
		int forward = candidates[i];
		assert(forward!=location);
		if (validateMove(location, forward, env)	){
			neighbors.emplace_back(forward);
		}
	}

}

#endif
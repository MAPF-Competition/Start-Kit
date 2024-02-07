#ifndef heuristics_hpp
#define heuristics_hpp

#include "SharedEnv.h"
#include "States.h"
#include "Types.h"
#include "utils.hpp"
#include <queue>


void compute_heuristics(const SharedEnvironment* env, HeuristicTable& my_heuristic, int goal_location)
{

	struct Node
	{
		int location;
		int direction;
		int value;

		Node() = default;
		Node(int location,int direction, int value) : location(location), direction(direction), value(value) {}
		// the following is used to compare nodes in the OPEN list
		struct compare_node
		{
			// returns true if n1 > n2 (note -- this gives us *min*-heap).
			bool operator()(const Node& n1, const Node& n2) const
			{
				return n1.value >= n2.value;
			}
		};  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)
	};
	
	// initialize my_heuristic, but have error on malloc: Region cookie corrupted for region
	my_heuristic.resize(env->map.size(),MAX_TIMESTEP);

	// generate a heap that can save nodes (and a open_handle)
	std::queue<Node> heap;
	Node root(goal_location,0, 0);
	my_heuristic[goal_location] = 0;
	heap.push(root);  // add root to heap
	
	std::vector<int> neighbors;
	while (!heap.empty())
	{
		Node curr = heap.front();
		heap.pop();

		getNeighborLocs(env,neighbors,curr.location);
		for (int next : neighbors)
		{
			assert(next >= 0 && next < env->map.size());
			//set current cost for reversed direction
			if (my_heuristic[next]>curr.value+1)
			{
				my_heuristic[next] = curr.value+1;
				heap.emplace(next,0, curr.value+1);
			}
		}
	}
}



void compute_heuristics(const SharedEnvironment* env, HeuristicTable& my_heuristic, std::vector<int>& traffic, int goal_location)
{

	struct Node
	{
		int location;
		int direction;
		int value;

		Node() = default;
		Node(int location,int direction, int value) : location(location), direction(direction), value(value) {}
		// the following is used to compare nodes in the OPEN list
		struct compare_node
		{
			// returns true if n1 > n2 (note -- this gives us *min*-heap).
			bool operator()(const Node& n1, const Node& n2) const
			{
				return n1.value >= n2.value;
			}
		};  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)
	};
	
	// initialize my_heuristic, but have error on malloc: Region cookie corrupted for region
	my_heuristic.resize(env->map.size(),MAX_TIMESTEP);

	// generate a heap that can save nodes (and a open_handle)
	std::queue<Node> heap;
	Node root(goal_location,0, 0);
	my_heuristic[goal_location] = 0;
	heap.push(root);  // add root to heap
	
	std::vector<int> neighbors;
	while (!heap.empty())
	{
		Node curr = heap.front();
		heap.pop();
        // if (traffic[curr.location] == -1)
		    getNeighborLocs(env,neighbors,curr.location);
        // else {
        //     neighbors.clear();
        //     int d = (traffic[curr.location]+2)%4;
        //     int candidates[4] = { curr.location + 1,curr.location + env->cols, curr.location - 1, curr.location - env->cols};
        //     neighbors.push_back(candidates[d]);
        // }

		for (int next : neighbors)
		{
			if (traffic[next] != -1 ){
				int d = traffic[next];
				int diff = curr.location - next;
				if (
					(diff == 1 && d !=0) ||
					(diff == -1 && d !=2) ||
					(diff == env->cols && d !=1) ||
					(diff == -env->cols && d !=3)
				)
					continue;
			}

			if (traffic[curr.location] != -1 ){
				int d = traffic[curr.location];
    			int candidates[4] = { curr.location + 1,curr.location + env->cols, curr.location - 1, curr.location - env->cols};
				if (
					next == candidates[d]
				)
					continue;
			}

            
			assert(next >= 0 && next < env->map.size());
			//set current cost for reversed direction
			if (my_heuristic[next]>curr.value+1)
			{
				my_heuristic[next] = curr.value+1;
				heap.emplace(next,0, curr.value+1);
			}
		}
	}
}


#endif
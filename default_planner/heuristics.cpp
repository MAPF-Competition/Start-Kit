
#include "heuristics.h"
#include <queue>

namespace DefaultPlanner{

std::vector<HeuristicTable> global_heuristictable;
Neighbors global_neighbors;



void init_neighbor(SharedEnvironment* env){
	global_neighbors.resize(env->rows * env->cols);
	for (int row=0; row<env->rows; row++){
		for (int col=0; col<env->cols; col++){
			int loc = row*env->cols+col;
			if (env->map[loc]==0){
				if (row>0 && env->map[loc-env->cols]==0){
					global_neighbors[loc].push_back(loc-env->cols);
				}
				if (row<env->rows-1 && env->map[loc+env->cols]==0){
					global_neighbors[loc].push_back(loc+env->cols);
				}
				if (col>0 && env->map[loc-1]==0){
					global_neighbors[loc].push_back(loc-1);
				}
				if (col<env->cols-1 && env->map[loc+1]==0){
					global_neighbors[loc].push_back(loc+1);
				}
			}
		}
	}
};

void init_heuristics(SharedEnvironment* env){
	if (global_heuristictable.size()==0){
		global_heuristictable.resize(env->map.size());
		init_neighbor(env);
	}

}

void init_heuristic(HeuristicTable& ht, SharedEnvironment* env, int goal_location){
	// initialize my_heuristic, but have error on malloc: Region cookie corrupted for region
	ht.htable.clear();
	ht.htable.resize(env->map.size(),MAX_TIMESTEP);
	ht.open.clear();
	// generate a open that can save nodes (and a open_handle)
	HNode root(goal_location,0, 0);
	ht.htable[goal_location] = 0;
	ht.open.push_back(root);  // add root to open
}


int get_heuristic(HeuristicTable& ht, SharedEnvironment* env, int source, Neighbors* ns){
		if (ht.htable[source] < MAX_TIMESTEP) return ht.htable[source];

		std::vector<int> neighbors;
		int cost, diff;
		while (!ht.open.empty())
		{
			HNode curr = ht.open.front();
			ht.open.pop_front();

			
			getNeighborLocs(ns,neighbors,curr.location);

			
			for (int next : neighbors)
			{
				cost = curr.value + 1;
				diff = curr.location - next;
				
				assert(next >= 0 && next < env->map.size());
				//set current cost for reversed direction

				if (cost >= ht.htable[next] )
					continue;

				ht.open.emplace_back(next,0, cost);
				ht.htable[next] = cost;
				
			}

			if (source == curr.location)
				return curr.value;
		}


		return MAX_TIMESTEP;
}

int get_h(SharedEnvironment* env, int source, int target){
	if (global_heuristictable.empty()){
		init_heuristics(env);
	}

	if (global_heuristictable.at(target).empty()){
		init_heuristic(global_heuristictable.at(target),env,target);
	}

	return get_heuristic(global_heuristictable.at(target), env, source, &global_neighbors);
}



void init_dist_2_path(Dist2Path& dp, SharedEnvironment* env, Traj& path){
	if (dp.dist2path.empty())
		dp.dist2path.resize(env->map.size(), d2p(0,-1,MAX_TIMESTEP,MAX_TIMESTEP));
	
	dp.open.clear();
	dp.label++;

    int togo = 0;
    for(int i = path.size()-1; i>=0; i--){
        auto p = path[i];
		assert(dp.dist2path[p].label != dp.label || dp.dist2path[p].cost == MAX_TIMESTEP);
		dp.open.emplace_back(dp.label,p,0,togo);
		dp.dist2path[p] = {dp.label,p,0,togo};
		togo++;
    }

}

std::pair<int,int> get_source_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns)
{
	if (dp.dist2path[source].label == dp.label && dp.dist2path[source].cost < MAX_TIMESTEP){
		// std::cout<<dp.dist2path[source].first<<" "<<dp.dist2path[source].second<<std::endl;

		return std::make_pair(dp.dist2path[source].cost, dp.dist2path[source].togo);
	}

	
	std::vector<int> neighbors;
	int cost;

	while (!dp.open.empty())
	{
		d2p curr = dp.open.front();
		dp.open.pop_front();



		getNeighborLocs(ns,neighbors,curr.id);

		for (int next_location : neighbors)
		{

			cost = curr.cost + 1;

			if (dp.dist2path[next_location].label == dp.label && cost >= dp.dist2path[next_location].cost )
				continue;
			dp.open.emplace_back(dp.label,next_location,cost,curr.togo);
			dp.dist2path[next_location] = {dp.label,next_location,cost,curr.togo};
			
		}
		if (source == curr.id){
			// std::cout<<curr.second.first<<" "<<curr.second.second<<std::endl;
			return std::make_pair(curr.cost, curr.togo);
		}
	}

	return std::make_pair(MAX_TIMESTEP,0);
}

int get_dist_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns)
{

	std::pair<int, int> dists = get_source_2_path(dp,env, source, ns);

	return dists.first + dists.second;
}



}




#include "pibt.h"





namespace DefaultPlanner{

int get_gp_h(TrajLNS& lns, int ai, int target){
    int min_heuristic;

    if (!lns.traj_dists.empty() && !lns.traj_dists[ai].empty())
        min_heuristic = get_dist_2_path(lns.traj_dists[ai], lns.env, target, &(lns.neighbors));	
    else if (!lns.heuristics[lns.tasks.at(ai)].empty())
        min_heuristic = get_heuristic(lns.heuristics[lns.tasks.at(ai)], lns.env, target, &(lns.neighbors));
    else
        min_heuristic = manhattanDistance(target,lns.tasks.at(ai),lns.env);
    
    return min_heuristic;
}

bool causalPIBT(int curr_id, int higher_id,std::vector<State>& prev_states,
	 std::vector<State>& next_states,
      std::vector<int>& prev_decision, std::vector<int>& decision, 
	  std::vector<bool>& occupied, TrajLNS& lns
	  ){
	// The PIBT works like a causal PIBT when using MAPF-T model. But a normal PIBT when using MAPF model.
	
	assert(next_states[curr_id].location == -1);
    int prev_loc = prev_states[curr_id].location;
	int prev_orientation = prev_states[curr_id].orientation;
	int next[4] = { prev_loc + 1,prev_loc + lns.env->cols, prev_loc - 1, prev_loc - lns.env->cols};
	int orien_next_v = next[prev_orientation];

	assert(prev_loc >= 0 && prev_loc < lns.env->map.size());

	int target = lns.tasks.at(curr_id);

	// for each neighbor of (prev_loc,prev_direction), and a wait copy of current location, generate a successor
	std::vector<int> neighbors;
	std::vector<PIBT_C> successors;
	getNeighborLocs(&(lns.neighbors),neighbors,prev_loc);
	for (auto& neighbor: neighbors){

		assert(validateMove(prev_loc, neighbor, lns.env));

		int min_heuristic = get_gp_h(lns, curr_id, neighbor);

		successors.emplace_back(neighbor,min_heuristic,-1,rand());
	}

	int wait_heuristic = get_gp_h(lns, curr_id, prev_loc);

	successors.emplace_back(prev_loc, wait_heuristic,-1,rand());


	std::sort(successors.begin(), successors.end(), 
		[&](PIBT_C& a, PIBT_C& b)
		{
			int diff[4] = {1,lns.env->cols,-1,-lns.env->cols};
			if (a.heuristic == b.heuristic){
					//tie break on prefer moving forward
					if (a.location==orien_next_v && b.location!=orien_next_v)
						return true;
					if (a.location!=orien_next_v && b.location==orien_next_v)
						return false;
					// random tie break
					return a.tie_breaker < b.tie_breaker;
			}
			return a.heuristic < b.heuristic; 
		});


    for (auto& next: successors){
		if(occupied[next.location])
			continue;
		assert(validateMove(prev_loc, next.location, lns.env));
		
		if (next.location == -1)
			continue;
		if (decision[next.location] != -1){
			continue;
		}
		if (higher_id != -1 && prev_decision[next.location] == higher_id){
			continue;
		}
		next_states.at(curr_id) = State(next.location, -1, -1);
		decision.at(next.location) = curr_id;

        if (prev_decision.at(next.location) != -1 && 
			next_states.at(prev_decision.at(next.location)).location == -1){
            int lower_id = prev_decision.at(next.location);
            if (!causalPIBT(lower_id,curr_id,prev_states,next_states, prev_decision,decision, occupied,lns)){
				continue;
            }
        }

        return true;
    }

    next_states.at(curr_id) = State(prev_loc,-1 ,-1);;
    decision.at(prev_loc) = curr_id;     

	#ifndef NDEBUG
		std::cout<<"false: "<< next_states[curr_id].location<<","<<next_states[curr_id].orientation <<std::endl;
	#endif   

    return false;
}

Action getAction(State& prev, State& next){
	if (prev.location == next.location && prev.orientation == next.orientation){
		return Action::W;
	}
	if (prev.location != next.location && prev.orientation == next.orientation){
		return Action::FW;
	}
	if (next.orientation  == (prev.orientation+1)%4){
		return Action::CR;
	}
	if (next.orientation  == (prev.orientation+3)%4){
		return Action::CCR;
	}
	assert(false);
	return Action::W;
}

Action getAction(State& prev, int next_loc, SharedEnvironment* env){
	if (prev.location == next_loc){
		return Action::W;
	}
	int diff = next_loc -prev.location;
	int orientation;
	if (diff == 1){
		orientation = 0;
	}
	if (diff == -1){
		orientation = 2;
	}
	if (diff == env->cols){
		orientation = 1;
	}
	if (diff == -env->cols){
		orientation = 3;
	}
	if (orientation == prev.orientation){
		return Action::FW;
	}
	if (orientation  == (prev.orientation+1)%4){
		return Action::CR;
	}
	if (orientation  == (prev.orientation+3)%4){
		return Action::CCR;
	}
	if (orientation  == (prev.orientation+2)%4){
		return Action::CR;
	}
	assert(false);



}

bool moveCheck(int id, std::vector<bool>& checked,
		std::vector<DCR>& decided, std::vector<Action>& actions, std::vector<int>& prev_decision){
	if (checked.at(id) && actions.at(id) == Action::FW)
		return true;
	checked.at(id) = true;

	if (actions.at(id) != Action::FW)
		return false;

	//move forward
	int target = decided.at(id).loc;
	assert(target != -1);

	int na = prev_decision[target];
	if (na == -1)
		return true;

	if (moveCheck(na,checked,decided,actions,prev_decision))
		return true;
	actions.at(id) = Action::W;
	return false;
	

	

	
	
}

}
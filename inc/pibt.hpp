#ifndef pibt_hpp
#define pibt_hpp

#include <string>
#include <vector>
#include <list>
#include <unordered_set>
#include <tuple>
#include "ActionModel.h"
#include "SharedEnv.h"
#include "States.h"
#include "Types.h"
#include "utils.hpp"


bool constraintPIBT(int curr_id, int higher_id,std::vector<State>& prev_states,
	 std::vector<State>& next_states, const SharedEnvironment* env,
      vector<int>& prev_decision, vector<int>& decision, 
	  std::vector<int>& tasks, const std::vector<HeuristicTable>& heuristics,
	  std::vector<bool>& occupied, std::vector<int>& traffic){
	
	//assert next states of curr_id is not decided
	assert(next_states[curr_id].location == -1);
    int prev_loc = prev_states[curr_id].location;
	int prev_orientation = prev_states[curr_id].orientation;
	int next[4] = { prev_loc + 1,prev_loc + env->cols, prev_loc - 1, prev_loc - env->cols};
	int orien_next = next[prev_orientation];

	assert(prev_loc >= 0 && prev_loc < env->map.size());
	assert(prev_orientation >= 0 && prev_orientation < 4);

	// for each neighbor of (prev_loc,prev_direction), and a wait copy of current location, generate a successor
	std::vector<int> neighbors;
	std::vector<State> successors;
	getNeighborLocs(env,neighbors,prev_loc);
	for (auto& neighbor: neighbors){

		//check if prev_loc -> neighbor violoate the traffic rule on neighbor
		//violate if  traffic[neighbor] leads to prev_loc
		if (traffic[neighbor] != -1 ){
			int candidates[4] = { neighbor + 1,neighbor + env->cols, neighbor - 1, neighbor - env->cols};
			if (prev_loc  == candidates[traffic[neighbor]])
				continue;
		}
		assert(validateMove(prev_loc, neighbor, env));

		//get ,min(heuristics[tasks[curr_id]][neighbor].d)
		int min_heuristic;
		if (heuristics.at(tasks.at(curr_id)).empty())
			min_heuristic = manhattanDistance(neighbor,tasks.at(curr_id),env);
		else
			min_heuristic = heuristics.at(tasks.at(curr_id)).at(neighbor);
		successors.emplace_back(neighbor,min_heuristic,-1);
	}	
	successors.emplace_back(prev_loc,
		heuristics.at(tasks.at(curr_id)).empty()? 
			manhattanDistance(prev_loc,tasks.at(curr_id),env):
			heuristics.at(tasks.at(curr_id)).at(prev_loc),
		-1);
	// std::sort(successors.begin(), successors.end(), 
	quickSort(successors,0, successors.size()-1, 
		[&](State& a, State& b)
		{
			int diff[4] = {1,env->cols,-1,-env->cols};
			if (a.timestep == b.timestep){
				// if (a==orien_next && b!=orien_next)
				// 	return true;
				// if (a!=orien_next && b==orien_next)
				// 	return false;
				return rand()%2==1;
			}
			return a.timestep < b.timestep; 
		});

#ifndef NDEBUG
	std::cout<<"curr_id: "<<curr_id<<" prev_loc: "<<prev_loc<<" prev_orientation: "<<prev_orientation<<","<< prev_states[curr_id].timestep<<std::endl;
	for (auto& next: successors){
		std::cout<<curr_id <<" next: "<<next.location<<" "<<next.timestep<<" "<<next.orientation<<std::endl;
	}
	if (!heuristics.at(tasks.at(curr_id)).empty() && tasks.at(curr_id) != prev_loc)
		assert(successors.front().location!=prev_loc);
#endif

    for (auto& next: successors){
		if (occupied[next.location])
			continue;
		
		assert(validateMove(prev_loc, next.location, env));
		
		if (next.location == -1)
			continue;
		if (decision[next.location] != -1){
			continue;
		}
		if (higher_id != -1 && prev_decision[next.location] == higher_id){
			continue;
		}

		next_states.at(curr_id) = next;
		decision.at(next.location) = curr_id;

        if (prev_decision.at(next.location) != -1 && 
			next_states.at(prev_decision.at(next.location)).location == -1)
			// next_states.at(prev_decision.at(prev_states[curr_id].location)).location == -1 )
			{
            int lower_id = prev_decision.at(next.location);
            if (!constraintPIBT(lower_id,curr_id,prev_states,next_states,env, prev_decision,decision,tasks,heuristics, occupied, traffic)){
				continue;
            }
					// next_states.at(curr_id) = prev_states.at(curr_id); // Wait after pushing
        }

		#ifndef NDEBUG
		std::cout<<"true: "<< next.location<<","<<next.orientation <<std::endl;
		#endif
        return true;
    }

    next_states.at(curr_id) = State(prev_loc,-1 ,-1);
    decision.at(prev_loc) = curr_id;     

	#ifndef NDEBUG
		std::cout<<"false: "<< next_states[curr_id].location<<","<<next_states[curr_id].orientation <<std::endl;
	#endif   

    return false;
}


inline Action getAction(State& prev, State& next){
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

inline Action getAction(State& prev, int next_loc, SharedEnvironment* env){
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
	std::cout << prev << "|" << std::to_string(next_loc) << std::endl;
	assert(false);
	// return Action::W;

}

inline bool moveCheck(int id, std::vector<bool>& checked,
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

#endif

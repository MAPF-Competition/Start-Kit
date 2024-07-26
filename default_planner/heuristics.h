
#ifndef heuristics_hpp
#define heuristics_hpp

#include "Types.h"
#include "utils.h"
#include <queue>
#include "TrajLNS.h"
#include "search_node.h"

namespace TrafficMAPF{

void init_heuristic(HeuristicTable& ht, SharedEnvironment* env, int goal_location);

int get_heuristic(HeuristicTable& ht, SharedEnvironment* env, std::vector<Int4>& flow, int source, Neighbors* ns);


void init_dist_2_path(Dist2Path& dp, SharedEnvironment* env, Traj& path);

std::pair<int,int> get_source_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns);

int get_dist_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns);

}
#endif
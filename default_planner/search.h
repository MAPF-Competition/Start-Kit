
#ifndef search_hpp
#define search_hpp

#include "Types.h"
#include "utils.h"
#include "Memory.h"
#include "heap.h"
#include "search_node.h"
#include "heuristics.h"

namespace DefaultPlanner{
//a astar minimized the opposide traffic flow with existing traffic flow

s_node astar(SharedEnvironment* env, std::vector<Int4>& flow,
    HeuristicTable& ht, Traj& traj,
    MemoryPool& mem, int start, int goal, Neighbors* ns);
}

#endif
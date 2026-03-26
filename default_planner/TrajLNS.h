#ifndef TRAJ_LNS_H
#define TRAJ_LNS_H

#include "Types.h"
#include "Memory.h"
#include "search_node.h"
#include "heap.h"
#include "heuristics.h"
#include <iostream>

#include <set>

namespace DefaultPlanner{
// enum ADAPTIVE {RANDOM, CONGESTION, COUNT};
enum ADAPTIVE {RANDOM, CONGESTION, DEVIATION, COUNT};

// extern std::vector<HeuristicTable> global_heuristictable;
extern std::unordered_map<int, HeuristicTable> global_heuristictable;
extern bool global_heuristics_initialized;
extern Neighbors global_neighbors;

struct FW_Metric{
    int id;
    int deviation;
    int last_replan_t;
    int rand;

    FW_Metric(int i, int d, int l) : id(i), deviation(d),last_replan_t(l){};
    FW_Metric(){};
};

struct FlowHeuristic{
    HeuristicTable* h; 
    int target;
    int origin;
    pqueue_min_of open;
    MemoryPool mem;


    bool empty(){
        return mem.generated() == 0;
    }
    void reset(){
        // op_flows.clear();
        // depths.clear();
        // dists.clear();
        open.clear();
        mem.reset();
    }

};

class TrajLNS{
    public:
    SharedEnvironment* env;
    std::vector<int> tasks;

    TimePoint start_time;
    int t_ms=0;

    std::vector<Traj> trajs;

    std::vector<std::pair<int,int>> deviation_agents;

    std::vector<Int4> flow;
    // std::vector<HeuristicTable>& heuristics;
    std::unordered_map<int, HeuristicTable>& heuristics;
    std::vector<Dist2Path> traj_dists;
    std::vector<s_node> goal_nodes;// store the goal node of single agent search for each agent. contains all cost information.

    std::vector<FW_Metric> fw_metrics;
    Neighbors& neighbors;


    int traj_inited = 0;
    int dist2path_inited = 0;
    int soc = 0;

    MemoryPool mem;

    void init_mem(){
        mem.init(env->map.size());
    }

    TrajLNS(SharedEnvironment* env, std::unordered_map<int, HeuristicTable>& heuristics, Neighbors& neighbors):
        env(env),
        trajs(env->num_of_agents),
        tasks(env->num_of_agents),
        flow(env->map.size(),Int4({0,0,0,0})), heuristics(heuristics),
        traj_dists(env->num_of_agents),goal_nodes(env->num_of_agents),
        fw_metrics(env->num_of_agents),neighbors(neighbors){
        };


    TrajLNS():heuristics(global_heuristictable), neighbors(global_neighbors){};

    size_t memory_usage(){
        // ZoneScoped;
        size_t total = 0;
        const double GB = 1000.0 * 1000.0 * 1000.0;
        // Heuristics
        size_t heuristics_mem = heuristics.size() * sizeof(HeuristicTable);
        for (const auto& [key, ht] : heuristics) {
            heuristics_mem += 2 * ht.htable.size() * sizeof(int);
            heuristics_mem += ht.open.size() * sizeof(HNode);
        }
        std::cout << "TrajLNS:heuristics_mem_GB = " << (static_cast<double>(heuristics_mem) / GB) << std::endl;
        total += heuristics_mem;
        // Flow
        size_t flow_mem = flow.capacity() * sizeof(Int4);
        std::cout << "TrajLNS:flow_mem_GB = " << (static_cast<double>(flow_mem) / GB) << std::endl;
        total += flow_mem;
        // Traj dists
        size_t traj_dists_mem = traj_dists.capacity() * sizeof(Dist2Path);
        for (const auto& dp : traj_dists) {
            traj_dists_mem += dp.dist2path.size() * sizeof(d2p);
            traj_dists_mem += dp.open.size() * sizeof(d2p);
        }
        std::cout << "TrajLNS:traj_dists_mem_GB = " << (static_cast<double>(traj_dists_mem) / GB) << std::endl;
        total += traj_dists_mem;
        // Goal nodes
        size_t goal_nodes_mem = goal_nodes.capacity() * sizeof(s_node);
        for (const auto& node : goal_nodes) {
            goal_nodes_mem += sizeof(int) * 9; // assuming each s_node has 9 int
        }
        std::cout << "TrajLNS:goal_nodes_mem_GB = " << (static_cast<double>(goal_nodes_mem) / GB) << std::endl;
        total += goal_nodes_mem;
        // FW metrics
        size_t fw_metrics_mem = fw_metrics.capacity() * sizeof(FW_Metric);
        for (auto& metric : fw_metrics) {
            fw_metrics_mem += sizeof(int) * 4; // assuming each FW_Metric has 4
        }
        std::cout << "TrajLNS:fw_metrics_mem_GB = " << (static_cast<double>(fw_metrics_mem) / GB) << std::endl;
        total += fw_metrics_mem;
        // Trajs
        size_t trajs_mem = trajs.capacity() * sizeof(Traj);
        for (auto& traj : trajs) {
            trajs_mem += traj.capacity() * sizeof(int); // assuming each Traj is a vector of int
        }
        std::cout << "TrajLNS:trajs_mem_GB = " << (static_cast<double>(trajs_mem) / GB) << std::endl;
        total += trajs_mem;
        // Deviation agents (vector of pairs<int,int>)
        size_t deviation_agents_mem = deviation_agents.capacity() * sizeof(std::pair<int,int>);
        std::cout << "TrajLNS:deviation_agents_mem_GB = " << (static_cast<double>(deviation_agents_mem) / GB) << std::endl;
        total += deviation_agents_mem;
        // Internal memory pool (search nodes)
        size_t mem_pool_mem = mem.bytes();
        std::cout << "TrajLNS:mem_pool_mem_GB = " << (static_cast<double>(mem_pool_mem) / GB) << std::endl;
        total += mem_pool_mem;
        // Total
        std::cout << "TrajLNS:total_mem_GB = " << (static_cast<double>(total) / GB) << std::endl;
        return total;
    }
    

};
}
#endif
#ifndef TRAJ_LNS_H
#define TRAJ_LNS_H

#include "Types.h"
#include "Memory.h"
#include "search_node.h"
#include "heap.h"
#include <iostream>

#include <set>

namespace TrafficMAPF{
// enum ADAPTIVE {RANDOM, CONGESTION, COUNT};
enum ADAPTIVE {RANDOM, CONGESTION, DEVIATION, COUNT};

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
    std::vector<HeuristicTable> heuristics;
    std::vector<Dist2Path> traj_dists;
    std::vector<s_node> goal_nodes;// store the goal node of single agent search for each agent. contains all cost information.

    std::vector<FW_Metric> fw_metrics;
    Neighbors neighbors;


    int traj_inited = 0;
    int dist2path_inited = 0;
    int soc = 0;

    MemoryPool mem;

    void init_mem(){
        mem.init(env->map.size());
    }

    TrajLNS(SharedEnvironment* env):
        env(env),
        trajs(env->num_of_agents),
        tasks(env->num_of_agents),
        flow(env->map.size(),Int4({0,0,0,0})), heuristics(env->map.size()),
        traj_dists(env->num_of_agents),goal_nodes(env->num_of_agents),
        fw_metrics(env->num_of_agents){
            init_neighbor();
        };


    TrajLNS(){};

    void init_neighbor(){
        neighbors.resize(this->env->rows * this->env->cols);
        for (int row=0; row<this->env->rows; row++){
            for (int col=0; col<this->env->cols; col++){
                int loc = row*this->env->cols+col;
                if (this->env->map[loc]==0){
                    if (row>0 && this->env->map[loc-this->env->cols]==0){
                        neighbors[loc].push_back(loc-this->env->cols);
                    }
                    if (row<this->env->rows-1 && this->env->map[loc+this->env->cols]==0){
                        neighbors[loc].push_back(loc+this->env->cols);
                    }
                    if (col>0 && this->env->map[loc-1]==0){
                        neighbors[loc].push_back(loc-1);
                    }
                    if (col<this->env->cols-1 && this->env->map[loc+1]==0){
                        neighbors[loc].push_back(loc+1);
                    }
                }
            }
        }
    };

};
}
#endif
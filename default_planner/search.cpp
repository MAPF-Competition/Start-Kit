


#include "search.h"


namespace TrafficMAPF
{
std::chrono::nanoseconds t;
//a astar minimized the opposide traffic flow with existing traffic flow

s_node astar(SharedEnvironment* env, std::vector<Int4>& flow,
    HeuristicTable& ht, Traj& traj,
    MemoryPool& mem, int start, int start_direct, int goal, Neighbors* ns)
{
    mem.reset();

    int expanded=0;
    int generated=0;
    int h;

    if(ht.empty())
        h = manhattanDistance(start,goal,env);
    else
        h = get_heuristic(ht,env, start, ns);
    

    
    s_node* root = mem.generate_node(start*4+start_direct,0, h,0,0,0);

    if (start == goal)
    {
        traj.clear();
        traj.push_back(start*4+start_direct);
        return *root;
    }

    pqueue_min_of open;
    re_of re;

    open.push(root);

    int  diff, d, cost, op_flow, total_cross, all_vertex_flow,vertex_flow, depth,p_diff;
    int next_d1, next_d2, next_d1_loc, next_d2_loc;
    int temp_op, temp_vertex;
    double tie_breaker, decay_factor;

    s_node* goal_node = nullptr;
    // int neighbors[4];
    std::vector<std::pair<int,int>> neighbors;
    int next_neighbors[4];



    while (open.size() > 0)
    {
        s_node* curr = open.pop();
        curr->close();

        if (curr->id/4 == goal)
        {
            goal_node = curr;
            break;
        }
        expanded++;

        int curr_loc = curr->id/4;
        int curr_direct = curr->id%4;
        // getNeighborLocs(ns,neighbors,curr->id);
        getNeighbors(env,neighbors,curr_loc,curr_direct);
        
        for (auto next: neighbors)
        {

            cost = curr->g+1;
            tie_breaker = curr->tie_breaker;

            assert(next.first >= 0 && next.first < env->map.size());
            depth = curr->depth + 1;

            //moving direction
            //flow
            op_flow = 0;
            all_vertex_flow = 0;

            if(ht.empty())
                h = manhattanDistance(next.first,goal,env);
            else
                h = get_heuristic(ht,env, next.first, ns);

            diff = next.first - curr_loc;
            d = get_d(diff,env);


            temp_op = ( (flow[curr_loc].d[d]+1) * flow[next.first].d[(d+2)%4]);///( ( (flow[curr->id].d[d]+1) + flow[next].d[(d+2)%4]));

            //all vertex flow
            //the sum of all out going edge flow is the same as the total number of vertex visiting.
            temp_vertex = 1;
            for (int j=0; j<4; j++)
            {
                temp_vertex += flow[next.first].d[j];                
            }

            op_flow += temp_op;
        
            all_vertex_flow+= (temp_vertex-1) /2;

            p_diff = 0;
            if (curr->parent != nullptr)
            {
                p_diff = curr_loc - curr->parent->id/4;
            }

            op_flow += curr->op_flow; //op_flow is contra flow
            all_vertex_flow += curr->all_vertex_flow;

            int next_id = next.first*4+next.second;

            s_node temp_node(next_id,cost,h,op_flow, depth);
            temp_node.tie_breaker = tie_breaker;
            temp_node.set_all_flow(op_flow,  all_vertex_flow);

            if (!mem.has_node(next_id))
            {
                s_node* next_node = mem.generate_node(next_id,cost,h,op_flow, depth,all_vertex_flow);
                next_node->parent = curr;
                next_node->tie_breaker = tie_breaker;
                open.push(next_node);
                generated++;
            }
            else
            { 
                s_node* existing = mem.get_node(next_id);

                if (!existing->is_closed())
                {
                    if (re(temp_node,*existing))
                    {
                        existing->g = cost;
                        existing->parent = curr;
                        existing->depth = depth;
                        existing->tie_breaker = tie_breaker;
                        existing->set_all_flow(op_flow,  all_vertex_flow);
                        open.decrease_key(existing);
                    }
                }
                else{

                    if (re(temp_node,*existing))
                    { 
                        std::cout << "error in astar: re-expansion" << std::endl;
                        assert(false);
                        exit(1);
                    }

                } 
            }
        }
            

          
    }


    if (goal_node == nullptr){
        std::cout << "error in astar: no path found "<< start<<","<<goal << std::endl;
        assert(false);
        exit(1);
    }

    traj.resize(goal_node->depth+1);
    s_node* curr = goal_node;
    for (int i=goal_node->depth; i>=0; i--)
    {
        traj[i] = curr->id;
        curr = curr->parent;
    }

    return *goal_node;
}
}


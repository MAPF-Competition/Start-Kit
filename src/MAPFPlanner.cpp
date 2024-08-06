#include <random>
#include <Entry.h>

//default planner includes
#include "pibt.h"
#include "flow.h"
#include "heuristics.h"
#include "Types.h"
using namespace TrafficMAPF;


//default planner data
std::vector<int> decision; 
std::vector<int> prev_decision;
std::vector<double> p;
std::vector<State> prev_states;
std::vector<State> next_states;
std::vector<int> ids;
std::vector<double> p_copy;
std::vector<bool> occupied;
std::vector<DCR> decided;
std::vector<bool> checked;
std::vector<bool> task_change;
int RELAX = 200;
TrajLNS trajLNS;



struct AstarNode
{
    int location;
    int direction;
    int f,g,h;
    AstarNode* parent;
    int t = 0;
    bool closed = false;
    AstarNode(int _location,int _direction, int _g, int _h, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),parent(_parent) {}
    AstarNode(int _location,int _direction, int _g, int _h, int _t, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),t(_t),parent(_parent) {}
};


struct cmp
{
    bool operator()(AstarNode* a, AstarNode* b)
    {
        if(a->f == b->f) return a->g <= b->g;
        else return a->f > b->f;
    }
};



void MAPFPlanner::initialize(int preprocess_time_limit)
{
    assert(env->num_of_agents != 0);
    p.resize(env->num_of_agents);
    decision.resize(env->map.size(), -1);
    prev_states.resize(env->num_of_agents);
    next_states.resize(env->num_of_agents);
    decided.resize(env->num_of_agents,DCR({-1,DONE::DONE}));
    occupied.resize(env->map.size(),false);
    checked.resize(env->num_of_agents,false);
    ids.resize(env->num_of_agents);
    task_change.resize(env->num_of_agents,false);
    for (int i = 0; i < ids.size();i++){
        ids[i] = i;
    }

    trajLNS = TrajLNS(env);
    trajLNS.init_mem();

    std::shuffle(ids.begin(), ids.end(), std::mt19937(std::random_device()()));
    for (int i = 0; i < ids.size();i++){
        p[ids[i]] = ((double)(ids.size() - i))/((double)(ids.size()+1));
    }
    p_copy = p;


}


// plan using simple A* that ignores the time dimension
void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{
    TimePoint start_time = std::chrono::steady_clock::now();
    TimePoint init_heuristic_budget = start_time + std::chrono::milliseconds(time_limit*50/2);
    TimePoint end_time = start_time + std::chrono::milliseconds(time_limit*100);

    // auto get_median_makespan_of_unllo


    // cout<<"---timestep,"<< env->curr_timestep<<endl;
    prev_decision.clear();
    prev_decision.resize(env->map.size(), -1);
    occupied.clear();
    occupied.resize(env->map.size(),false);

    int count = 0;
    
    for(int i=0; i<env->num_of_agents; i++)
    {
        if ( (trajLNS.traj_inited < env->num_of_agents && std::chrono::steady_clock::now() < init_heuristic_budget) || (trajLNS.traj_inited >= env->num_of_agents)){
            for(int j=0; j<env->goal_locations[i].size(); j++)
            {
                int goal_loc = env->goal_locations[i][j].first;
                    if (trajLNS.heuristics.at(goal_loc).empty()){
                        init_heuristic(trajLNS.heuristics[goal_loc],env,goal_loc);
                        count++;
                    }
            }
        }

        assert(env->goal_locations[i].size()>0);
        task_change[i] =  env->goal_locations[i].front().first != trajLNS.tasks[i];
        trajLNS.tasks[i] = env->goal_locations[i].front().first;

        assert(env->curr_states[i].location >=0);
        prev_states[i] = env->curr_states[i];
        next_states[i] = State();
        prev_decision[env->curr_states[i].location] = i; 
        if (decided[i].loc == -1){
            decided[i].loc = env->curr_states[i].location;
            assert(decided[i].state == DONE::DONE);
        }
        if (prev_states[i].location == decided[i].loc){
            decided[i].state = DONE::DONE;
        }
        if (decided[i].state == DONE::NOT_DONE){
            occupied.at(decided[i].loc) = true;
            occupied.at(prev_states[i].location) = true;
        }

        if(task_change[i])
            p[i] = p_copy[i];
        else
            p[i] = p[i]+1;
        
    }

    bool init_done = trajLNS.traj_inited == env->num_of_agents;

    //task change
    for (int i = 0; i < env->num_of_agents;i++){
        if (task_change[i] && !trajLNS.trajs[i].empty()){
            remove_traj(trajLNS, i);
            update_traj(trajLNS, i);
        }
    }

    if (trajLNS.traj_inited < env->num_of_agents){
        init_traj(trajLNS, end_time);
    }

    std::unordered_set<int> updated;

    //endtime is start time + 0.5 second
    if (init_done)
        frank_wolfe(trajLNS, updated,end_time);


    std::sort(ids.begin(), ids.end(), [&](int a, int b) {
            return p.at(a) > p.at(b);
        }
    );

    //pibt
    for (int i : ids){
        if (decided[i].state == DONE::NOT_DONE){
            continue;
        }
        if (next_states[i].location==-1){
            assert(prev_states[i].location >=0 && prev_states[i].location < env->map.size());
            causalPIBT(i,-1,prev_states,next_states,
                prev_decision,decision,
                occupied, trajLNS);
        }
    }
    
    actions.resize(env->num_of_agents);
    for (int id : ids){

        if (next_states.at(id).location!= -1)
            decision.at(next_states.at(id).location) = -1;
        
        assert(
            (next_states.at(id).location >=0 && decided.at(id).state == DONE::DONE)||
            (next_states.at(id).location == -1 && decided.at(id).state == DONE::NOT_DONE)
        );

        if (next_states.at(id).location >=0){
            decided.at(id) = DCR({next_states.at(id).location,DONE::NOT_DONE});
        }
        actions.at(id) = getAction(prev_states.at(id),decided.at(id).loc, env);
        checked.at(id) = false;

    }

    for (int id=0;id < env->num_of_agents ; id++){
        if (!checked.at(id) && actions.at(id) == Action::FW){
            moveCheck(id,checked,decided,actions,prev_decision);
        }
    }



    prev_states = next_states;
    return;
}

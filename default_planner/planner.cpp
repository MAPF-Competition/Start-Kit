#include "planner.h"
#include "heuristics.h"
#include "SharedEnv.h"
#include "pibt.h"
#include "flow.h"
#include "const.h"
#include <deque>
#include <numeric>
#include <thread>
#include <chrono>


namespace DefaultPlanner{

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
    std::vector<bool> require_guide_path;
    std::vector<int> dummy_goals;
    TrajLNS trajLNS;
    std::mt19937 mt1;
    // history of recent PIBT runtimes in milliseconds (sliding window of last 10)
    std::deque<int> pibt_time_history;
    constexpr int PIBT_TIME_HISTORY_LEN = 10; // max number of samples to keep

    /**
     * @brief Default planner initialization
     * 
     * @param preprocess_time_limit time limit for preprocessing in milliseconds
     * @param env shared environment object
     * 
     * The initialization function initializes the default planner data structures and heuristics tables.
     */
    void initialize(int preprocess_time_limit, SharedEnvironment* env){
            //initialise all required data structures
            assert(env->num_of_agents != 0);
            p.resize(env->num_of_agents);
            decision.resize(env->map.size(), -1);
            prev_states.resize(env->num_of_agents);
            next_states.resize(env->num_of_agents);
            decided.resize(env->num_of_agents,DCR({-1,DONE::DONE}));
            occupied.resize(env->map.size(),false);
            checked.resize(env->num_of_agents,false);
            ids.resize(env->num_of_agents);
            require_guide_path.resize(env->num_of_agents,false);
            for (int i = 0; i < ids.size();i++){
                ids[i] = i;
            }

            // initialise the heuristics tables containers
            init_heuristics(env);
            mt1.seed(0);
            srand(0);

            new (&trajLNS) TrajLNS(env, global_heuristictable, global_neighbors);
            trajLNS.init_mem();

            //assign intial priority to each agent
            std::shuffle(ids.begin(), ids.end(), mt1);
            for (int i = 0; i < ids.size();i++){
                p[ids[i]] = ((double)(ids.size() - i))/((double)(ids.size()+1));
            }
            p_copy = p;

            // Report memory usage of initialized structures
            const double GB = 1024.0 * 1024.0 * 1024.0;
            auto bytes_vec_bool = [](const std::vector<bool>& v){ return (v.capacity() + 7) / 8; };
            size_t mem_p = p.capacity() * sizeof(double);
            size_t mem_p_copy = p_copy.capacity() * sizeof(double);
            size_t mem_decision = decision.capacity() * sizeof(int);
            size_t mem_prev_states = prev_states.capacity() * sizeof(State);
            size_t mem_next_states = next_states.capacity() * sizeof(State);
            size_t mem_ids = ids.capacity() * sizeof(int);
            size_t mem_occupied = bytes_vec_bool(occupied);
            size_t mem_decided = decided.capacity() * sizeof(DCR);
            size_t mem_checked = bytes_vec_bool(checked);
            size_t mem_require_guide_path = bytes_vec_bool(require_guide_path);
            size_t mem_dummy_goals = dummy_goals.capacity() * sizeof(int);
            size_t mem_prev_decision = prev_decision.capacity() * sizeof(int);
            // Include TrajLNS internal structures
            size_t mem_trajLNS = trajLNS.memory_usage();
            size_t total_mem = mem_p + mem_p_copy + mem_decision + mem_prev_states + mem_next_states +
                               mem_ids + mem_occupied + mem_decided + mem_checked + mem_require_guide_path +
                               mem_dummy_goals + mem_prev_decision + mem_trajLNS;
            std::cout << "DefaultPlanner:init:p_mem_GB = " << (mem_p / GB) << std::endl;
            std::cout << "DefaultPlanner:init:p_copy_mem_GB = " << (mem_p_copy / GB) << std::endl;
            std::cout << "DefaultPlanner:init:decision_mem_GB = " << (mem_decision / GB) << std::endl;
            std::cout << "DefaultPlanner:init:prev_states_mem_GB = " << (mem_prev_states / GB) << std::endl;
            std::cout << "DefaultPlanner:init:next_states_mem_GB = " << (mem_next_states / GB) << std::endl;
            std::cout << "DefaultPlanner:init:ids_mem_GB = " << (mem_ids / GB) << std::endl;
            std::cout << "DefaultPlanner:init:occupied_mem_GB = " << (mem_occupied / GB) << std::endl;
            std::cout << "DefaultPlanner:init:decided_mem_GB = " << (mem_decided / GB) << std::endl;
            std::cout << "DefaultPlanner:init:checked_mem_GB = " << (mem_checked / GB) << std::endl;
            std::cout << "DefaultPlanner:init:require_guide_path_mem_GB = " << (mem_require_guide_path / GB) << std::endl;
            std::cout << "DefaultPlanner:init:dummy_goals_mem_GB = " << (mem_dummy_goals / GB) << std::endl;
            std::cout << "DefaultPlanner:init:prev_decision_mem_GB = " << (mem_prev_decision / GB) << std::endl;
            std::cout << "DefaultPlanner:init:trajLNS_mem_GB = " << (mem_trajLNS / GB) << std::endl;
            std::cout << "DefaultPlanner:init:total_mem_GB = " << (total_mem / GB) << std::endl;
            // Sleep 10s so the output can be read before proceeding
            // std::this_thread::sleep_for(std::chrono::seconds(10));
            return;
    };

    /**
     * @brief Default planner plan function
     * 
     * @param time_limit time limit for planning in milliseconds
     * @param actions vector of actions to be populated by the planner
     * @param env shared environment object
     * 
     * The plan function is the main function of the default planner. 
     * It computes the actions for the agents based on the current state of the environment.
     * The function first checks assignments/goal location changes and perform the necessary updates.
     * It then computes and optimises traffic flow optimised guide paths for the agents.
     * Finally, it computes the actions for the agents using PIBT that follows the guide path heuristics and returns the actions.
     * Note that the default planner ignores the turning action costs, and post-processes turning actions as additional delays on top of original plan.
     */
    void plan(int time_limit,vector<Action> & actions, SharedEnvironment* env){
        // ZoneScoped;
        // calculate the time planner should stop optimsing traffic flows and return the plan.
        TimePoint start_time = std::chrono::steady_clock::now();
        // estimate PIBT time budget using running average of last PIBT runtimes (fallback to rule-of-thumb if no history)
        int pibt_time;
        if (!pibt_time_history.empty()){
            // take the max of the pibt_time
            pibt_time = *std::max_element(pibt_time_history.begin(), pibt_time_history.end());
            std::cout << "PIBT time from history: " << pibt_time << " ms" << std::endl;
        }else{
            // fallback heuristic based on number of agents (original behaviour)
            pibt_time = PIBT_RUNTIME_PER_100_AGENTS * env->num_of_agents/100;
        }
        // leave TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE ms for computing PIBT actions; ensure non-negative remaining time
        int remaining_ms = time_limit - pibt_time - TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE;
        if (remaining_ms < 0){
            // if average PIBT time exceeds available window, shrink PIBT budget proportionally but keep at least small positive flow optimisation window
            // clamp PIBT time so that remaining_ms becomes a small slice (5%) if possible
            pibt_time = PIBT_RUNTIME_PER_100_AGENTS * env->num_of_agents/100;
            remaining_ms = std::max(0, time_limit - pibt_time - TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE);
            std::cout << "PIBT time clamped to: " << pibt_time << " ms" << std::endl;
        }
        std::cout << "Remaining time for traffic flow optimisation: " << remaining_ms << " ms" << std::endl;
        TimePoint end_time = start_time + std::chrono::milliseconds(remaining_ms);

        // recrod the initial location of each agent as dummy goals in case no goal is assigned to the agent.
        if (env->curr_timestep == 0){
            dummy_goals.resize(env->num_of_agents);
            for(int i=0; i<env->num_of_agents; i++)
            {
                dummy_goals.at(i) = env->curr_states.at(i).location;
            }
        }

        // data sturcture for record the previous decision of each agent
        prev_decision.clear();
        prev_decision.resize(env->map.size(), -1);

        // update the status of each agent and prepare for planning
        int count = 0;
        for(int i=0; i<env->num_of_agents; i++)
        {
            //initialise the shortest distance heuristic table for the goal location of the agent
            // if ( ( std::chrono::steady_clock::now() < end_time) ){
            //     for(int j=0; j<env->goal_locations[i].size(); j++)
            //     {
            //         int goal_loc = env->goal_locations[i][j].first;
            //         if (trajLNS.heuristics.find(goal_loc) == trajLNS.heuristics.end()){
            //             // init_heuristic(trajLNS.heuristics[goal_loc],env,goal_loc);
            //             count++;
            //         }
            //     }
            // }
            

            // set the goal location of each agent
            if (env->goal_locations[i].empty()){
                trajLNS.tasks[i] = dummy_goals.at(i);
                p[i] = p_copy[i];
            }
            else{
                trajLNS.tasks[i] = env->goal_locations[i].front().first;
            }

            // check if the agent need a guide path update, when the agent has no guide path or the guide path does not end at the goal location
            require_guide_path[i] = false;
            if (trajLNS.trajs[i].empty() || trajLNS.trajs[i].back() != trajLNS.tasks[i])
                require_guide_path[i] = true;
            
            // check if the agent completed the action in the previous timestep
            // if not, the agent is till turning towards the action direction, we do not need to plan new action for the agent
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
                decision.at(decided[i].loc) = i;
                next_states[i] = State(decided[i].loc,-1,-1);
            }

            // reset the pibt priority if the agent reached prvious goal location and switch to new goal location
            if(require_guide_path[i])
                p[i] = p_copy[i];
            else if (!env->goal_locations[i].empty())
                p[i] = p[i]+1;

            // give priority bonus to the agent if the agent is in a deadend location
            if (!env->goal_locations[i].empty() && trajLNS.neighbors[env->curr_states[i].location].size() == 1){
                p[i] = p[i] + 10;
            }
            
        }

        // compute the congestion minimised guide path for the agents that need guide path update
        for (int i = 0; i < env->num_of_agents;i++){
            if (std::chrono::steady_clock::now() > end_time)
            {
                std::cout << "Time limit reached during guide path computation" << i << "/" << env->num_of_agents << std::endl;
                break;
            }
            if (require_guide_path[i]){
                std::vector<int> old_traj = trajLNS.trajs[i];
                if (!trajLNS.trajs[i].empty())
                    remove_traj(trajLNS, i);
                if (!update_traj(trajLNS, i, &end_time)){
                    trajLNS.trajs[i] = old_traj;
                    if (!old_traj.empty()){
                        add_traj(trajLNS, i);
                    }
                    std::cout << "Time limit reached during guide path replanning " << i << "/" << env->num_of_agents << std::endl;
                    break;
                }
            }
            // size_t memory_usage = trajLNS.memory_usage();
        }

        // iterate and recompute the guide path to optimise traffic flow
        std::unordered_set<int> updated;
        frank_wolfe(trajLNS, updated,end_time);
        int ms_left_after_fw = std::max(
            0,
            (int) std::chrono::duration_cast<std::chrono::milliseconds>(
                end_time - std::chrono::steady_clock::now()).count());
        std::cout << "Remaining time after Frank-Wolfe: " << ms_left_after_fw << " ms" << std::endl;
        std::cout << "Frank-Wolfe finished, now PIBT" << std::endl;

        TimePoint pibt_start_time = std::chrono::steady_clock::now();
        // sort agents based on the current priority
        std::sort(ids.begin(), ids.end(), [&](int a, int b) {
                return p.at(a) > p.at(b);
            }
        );

        // compute the targeted next location for each agent using PIBT
        int pibt_cnt = 0;
        int ids_size = ids.size();
        for (int i : ids){
            if (decided[i].state == DONE::NOT_DONE){
                continue;
            }
            if (next_states[i].location==-1){
                assert(prev_states[i].location >=0 && prev_states[i].location < env->map.size());
                causalPIBT(i,-1,prev_states,next_states,
                    prev_decision,decision,
                    occupied, trajLNS);
                // std::cout << "PIBT finished for agent " << i << std::endl;
                // std::cout << "Progress: "
                //           << ++pibt_cnt << "/" << ids_size
                //           << " | dist2path=" << PIBT_CNT_get_dist_2_path
                //           << " | heuristic=" << PIBT_CNT_get_heuristic
                //           << " | manhattan=" << PIBT_CNT_manhattan
                //           << std::endl;
            }
        }
        std::cout << "PIBT finished for all agents" << std::endl;

        // post processing the targeted next location to turning or moving actions
        actions.resize(env->num_of_agents);
        for (int id : ids){
            //clear the decision table based on which agent has next_states
            if (next_states.at(id).location!= -1)
                decision.at(next_states.at(id).location) = -1;
            // if agent is newly assigned a targeted next location, record the decision as not done yet
            if (next_states.at(id).location >=0){
                decided.at(id) = DCR({next_states.at(id).location,DONE::NOT_DONE});
            }

            // post process the targeted next location to turning or moving actions
            actions.at(id) = getAction(prev_states.at(id),decided.at(id).loc, env);
            checked.at(id) = false;

        }

        // recursively check if the FW action can be executed by checking whether all agents in the front of the agent can move forward
        // if any agent cannot move foward due to turning, all agents behind the turning agent will not move forward.
        for (int id=0;id < env->num_of_agents ; id++){
            if (!checked.at(id) && actions.at(id) == Action::FW){
                moveCheck(id,checked,decided,actions,prev_decision);
            }
        }
        
        TimePoint pibt_end_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> pibt_duration = pibt_end_time - pibt_start_time;
        // record PIBT runtime in ms and maintain sliding window
        int pibt_ms = (int) std::chrono::duration_cast<std::chrono::milliseconds>(pibt_end_time - pibt_start_time).count() + 1;
        pibt_time_history.push_back(pibt_ms);
        if (pibt_time_history.size() > PIBT_TIME_HISTORY_LEN) pibt_time_history.pop_front();
        // (optional) print running average for debugging
        long long dbg_sum = 0; for(int t: pibt_time_history) dbg_sum += t;
        double avg_ms = (double)dbg_sum / pibt_time_history.size();
        prev_states = next_states;
        size_t memory_usage = trajLNS.memory_usage();
        std::cout << "PIBT running average (" << pibt_time_history.size() << "/" << PIBT_TIME_HISTORY_LEN << ") = " << avg_ms << " ms" << std::endl;
        // std::cout << "TrajLNS Memory Usage in GB: " << static_cast<double>(memory_usage) / (1024 * 1024 * 1024) << std::endl;
        return;

    };
}

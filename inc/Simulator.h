#include "ActionModel.h"
#include "SharedEnv.h"
#include "States.h"
#include "Executor.h"
#include "nlohmann/json.hpp"


//simulator is use for simulating the moves and store the current states
//has function (1) simulate move (based on the given actions and current states)
//             (2) sync share env?

class Simulator
{
public:
    Simulator(Grid &grid, std::vector<int>& start_locs, ActionModelWithRotate* model, Executor* executor = nullptr):
        map(grid), model(model), executor(executor)
    {
        num_of_agents = start_locs.size();
        starts.resize(num_of_agents);
        paths.resize(num_of_agents);

        for (size_t i = 0; i < start_locs.size(); i++)
            {
                if (grid.map[start_locs[i]] == 1)
                {
                    cout<<"error: agent "<<i<<"'s start location is an obstacle("<<start_locs[i]<<")"<<endl;
                    exit(0);
                }
                starts[i] = State(start_locs[i], 0, 0);
        }

        curr_states = starts;

        actual_movements.resize(num_of_agents);
        planner_movements.resize(num_of_agents);
        // prepare staged actions container for each agent
        staged_actions.resize(num_of_agents);

        // if no executor provided, create a default one (its env will be set later via sync_shared_env)
        if (this->executor == nullptr)
        {
            this->executor = new Executor();
        }
    }

    virtual ~Simulator()
    {
        if (executor != nullptr)
        {
            delete executor;
        }
    };

    vector<State> process_new_plan(int sync_time_limit, int overtime_runtime, vector<Action>& plan);

    vector<State> move(int move_time_limit, vector<Action>& next_actions);

    vector<State> execute_actions(vector<Action>& actions);

    void validate_actions_with_delay(vector<Action>& actions);

    //void sync_shared_env(SharedEnvironment* env);

    vector<State> get_current_state(){ return curr_states; }

    int get_curr_timestep() {return timestep;}

    bool get_all_valid(){ return all_valid;}

    void sync_shared_env(SharedEnvironment* env);

    nlohmann::ordered_json actual_path_to_json() const;

    nlohmann::ordered_json planned_path_to_json() const;

    nlohmann::ordered_json starts_to_json() const;

    nlohmann::ordered_json action_errors_to_json() const;

    int get_number_errors() const {return model->errors.size();}

private:
    Grid map;

    ActionModelWithRotate* model;

    Executor* executor;


    // #timesteps for simulation
    int timestep = 0;

    std::vector<Path> paths;

    vector<State> starts;
    int num_of_agents;

    vector<State> curr_states;
    
    //TODO: We need a list of delay durations for each agent. Need to populate this vector when we got the problem generator.
    vector<int> delays;

    vector<list<Action>> actual_movements;
    vector<list<Action>> planner_movements;

    vector<vector<Action>> staged_actions;

    bool all_valid = true;
    
};
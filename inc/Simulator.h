#include "ActionModel.h"
#include "SharedEnv.h"
#include "States.h"
#include "Executor.h"
#include "DelayGenerator.h"
#include "nlohmann/json.hpp"
#include <memory>


//simulator is use for simulating the moves and store the current states
//has function (1) simulate move (based on the given actions and current states)
//             (2) sync share env?

class Simulator
{
public:
    Simulator(Grid &grid, std::vector<int>& start_locs, ActionModelWithRotate* model, Executor* executor = nullptr, int max_counter = 10):
        map(grid), model(model), executor(executor), max_counter(max_counter)
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
                starts[i] = State(start_locs[i], 0, 0, max_counter);
        }

        curr_states = starts;
        predict_states = starts;

        actual_movements.resize(num_of_agents);
        planner_movements.resize(num_of_agents);
        staged_actions.resize(num_of_agents);

        // if no executor provided, create a default one (its env will be set later via sync_shared_env)
        if (this->executor == nullptr)
        {
            this->executor = new Executor();
        }
        set_delay_enabled(true);
    }

    virtual ~Simulator()
    {
        if (executor != nullptr)
        {
            delete executor;
        }
    };

    bool initialise_executor(int preprocess_time_limit);

    void process_new_plan(int sync_time_limit, int overtime_runtime, Plan& plan) ;

    vector<State> move(int move_time_limit);
    void move_all_wait(int steps); 

    void simulate_delay();

    void validate_actions_with_delay(vector<Action>& actions);

    void set_delay_enabled(bool enabled);
    void set_staged_action_validation_enabled(bool enabled)
    {
        executor_validation = enabled;
    }
    void set_delay_generator(std::unique_ptr<DelayGenerator> generator)
    {
        delay_generator = std::move(generator);

        const int min_delay = delay_generator ? delay_generator->get_config().minDelay : -1;
        const int max_delay = delay_generator ? delay_generator->get_config().maxDelay : -1;
        for (int i = 0; i < num_of_agents; i++)
        {
            starts[i].delay.minDelay = min_delay;
            starts[i].delay.maxDelay = max_delay;
            curr_states[i].delay.minDelay = min_delay;
            curr_states[i].delay.maxDelay = max_delay;
            predict_states[i].delay.minDelay = min_delay;
            predict_states[i].delay.maxDelay = max_delay;
        }
    }

    vector<State> get_current_state(){ return curr_states; }

    int get_curr_timestep() {return timestep;}

    int get_max_counter() const {return max_counter;}
    
    int get_chunk_size() const {return chunk_size;}

    void sync_shared_env(SharedEnvironment* env);

    nlohmann::ordered_json actual_path_to_json() const;

    nlohmann::ordered_json planned_path_to_json() const;

    nlohmann::ordered_json starts_to_json() const;

    nlohmann::ordered_json action_errors_to_json() const;
    nlohmann::ordered_json delay_intervals_to_json() const
    {
        if (delay_generator == nullptr)
        {
            nlohmann::ordered_json empty = nlohmann::ordered_json::array();
            for (int i = 0; i < num_of_agents; i++)
            {
                empty.push_back(nlohmann::ordered_json::array());
            }
            return empty;
        }
        return delay_generator->delay_intervals_to_json();
    }

    int get_number_errors() const {return model->errors.size();}

    void record_planned_movements(Action action, int agent_id);
    void record_actual_movements(State state, Action action, int agent_id);

    void set_chunk(int size, int max_simulation)
    {
        chunk_size = size;
        int num_chunks = (max_simulation + chunk_size - 1) / chunk_size; // calculate the number of chunks needed
        chunked_actual_movements.resize(num_of_agents, vector<list<pair<Action, int>>>(num_chunks));
        chunked_planner_movements.resize(num_of_agents, vector<list<pair<Action, int>>>(num_chunks));
        chunked_planner_snapshot_states.resize(num_of_agents, vector<State>(num_chunks)); 
        chunked_actual_snapshot_states.resize(num_of_agents, vector<State>(num_chunks)); 
        current_planner_chunk_count.resize(num_of_agents, 0);
        current_planner_chunk_index.resize(num_of_agents, 0);
        current_actual_chunk_count.resize(num_of_agents, 0);
        current_actual_chunk_index.resize(num_of_agents, 0);  
    }

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
    vector<State> predict_states;

    vector<list<Action>> actual_movements;
    vector<list<Action>> planner_movements;
    vector<vector<list<pair<Action, int>>>> chunked_actual_movements;
    vector<vector<list<pair<Action, int>>>> chunked_planner_movements;
    vector<vector<State>> chunked_planner_snapshot_states;
    vector<vector<State>> chunked_actual_snapshot_states;

    int chunk_size = 100; // the size of each chunk for chunked movements and states
    vector<int> current_planner_chunk_count; // the current chunk count for recording movements and states
    vector<int> current_planner_chunk_index; // the current chunk index for recording movements and states
    vector<int> current_actual_chunk_count; // the current chunk count for recording movements and states
    vector<int> current_actual_chunk_index; // the current chunk index for recording movements and states

    vector<vector<Action>> staged_actions;

    std::unique_ptr<DelayGenerator> delay_generator;
    bool delay_enabled = true;
    bool executor_validation = true;
    
    int max_counter;
};

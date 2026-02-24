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
        predict_states = starts;

        actual_movements.resize(num_of_agents);
        planner_movements.resize(num_of_agents);
        // chunked_actual_movements.resize(num_of_agents);
        // chunked_planner_movements.resize(num_of_agents);
        // chunked_snapshot_states.resize(num_of_agents);
        // prepare staged actions container for each agent
        staged_actions.resize(num_of_agents);
        delays.resize(num_of_agents, 0);

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

    bool initialise_executor(int preprocess_time_limit);

    void process_new_plan(int sync_time_limit, int overtime_runtime, Plan& plan) ;

    vector<State> move(int move_time_limit, vector<Action>& next_actions);

    void simulate_delay();

    void validate_actions_with_delay(vector<Action>& actions);

    vector<State> get_current_state(){ return curr_states; }

    int get_curr_timestep() {return timestep;}

    void sync_shared_env(SharedEnvironment* env);

    nlohmann::ordered_json actual_path_to_json() const;

    nlohmann::ordered_json planned_path_to_json() const;

    nlohmann::ordered_json starts_to_json() const;

    nlohmann::ordered_json action_errors_to_json() const;

    int get_number_errors() const {return model->errors.size();}

    void set_delay_profile(const vector<pair<int, int>>& delay_ranges,
                           const vector<vector<pair<int, int>>>& delay_schedule)
    {
        if (!delay_ranges.empty() && static_cast<int>(delay_ranges.size()) != num_of_agents)
        {
            throw std::invalid_argument("Invalid delay profile: delay_ranges size must match number of agents");
        }

        this->delay_schedule = delay_schedule;

        if (!delay_ranges.empty())
        {
            for (int i = 0; i < num_of_agents; i++)
            {
                int min_d = std::max(0, delay_ranges[i].first);
                int max_d = std::max(0, delay_ranges[i].second);
                if (max_d < min_d)
                {
                    std::swap(min_d, max_d);
                }

                starts[i].delay.minDelay = min_d;
                starts[i].delay.maxDelay = max_d;
                curr_states[i].delay.minDelay = min_d;
                curr_states[i].delay.maxDelay = max_d;
                predict_states[i].delay.minDelay = min_d;
                predict_states[i].delay.maxDelay = max_d;
            }
        }
    }

    void record_planned_movements(Action action, int agent_id);
    void record_actual_movements(Action action, int agent_id);

    void set_chunk(int size, int max_simulation)
    {
        chunk_size = size;
        int num_chunks = (max_simulation + chunk_size - 1) / chunk_size; // calculate the number of chunks needed
        chunked_actual_movements.resize(num_of_agents, vector<list<pair<int, int>>>(num_chunks));
        chunked_planner_movements.resize(num_of_agents, vector<list<pair<int, int>>>(num_chunks));
        chunked_snapshot_states.resize(num_of_agents, vector<State>(num_chunks));   
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
    vector<int> delays;

    vector<list<Action>> actual_movements;
    vector<list<Action>> planner_movements;
    vector<vector<list<pair<int, int>>>> chunked_actual_movements;
    vector<vector<list<pair<int, int>>>> chunked_planner_movements;
    vector<vector<State>> chunked_snapshot_states;

    int chunk_size = 100; // the size of each chunk for chunked movements and states

    vector<vector<Action>> staged_actions;

    vector<vector<pair<int, int>>> delay_schedule;
};

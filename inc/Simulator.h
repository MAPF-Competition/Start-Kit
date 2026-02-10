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
    Simulator(Grid &grid, std::vector<int>& start_locs, ActionModelWithRotate* model, Executor* executor = nullptr, std::mt19937* MT = nullptr):
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

    void process_new_plan(int sync_time_limit,int overtime_runtime, vector<Action>& plan) ;

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

    void set_delay_seed(int seed, int min_delay = 0, int max_delay = 0, double delay_prob = 0.0) 
    { 
        if (min_delay < 0 || max_delay < 0 || delay_prob < 0.0 || delay_prob > 1.0)
        {
            throw std::invalid_argument("Invalid delay configuration: min_delay, max_delay should be non-negative and delay_prob should be in [0,1]");
        }
        if (max_delay < min_delay)
        {
            std::swap(max_delay, min_delay);
        }
        MT.seed(seed); 
        this->min_delay = min_delay;
        this->max_delay = max_delay;
        this->delay_p = std::clamp(delay_prob, 0.0, 1.0);

        delay_event_ = std::bernoulli_distribution(delay_p);
        delay_len_ = std::uniform_int_distribution<int>(min_delay, max_delay);
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

    vector<vector<Action>> staged_actions;

    // delay configurations
    std::mt19937 MT;
    int min_delay, max_delay = 0;
    double delay_p = 0.0;
    std::bernoulli_distribution delay_event_{0.0};
    std::uniform_int_distribution<int> delay_len_{0, 0};
};
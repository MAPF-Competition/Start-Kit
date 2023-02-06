#include <cmath>
#include "CompetitionSystem.h"
#include <boost/tokenizer.hpp>

CompetitionSystem::CompetitionSystem(MAPFPlanner* planner): planner(planner), env(planner->env){};


bool CompetitionSystem::load_map(string fname){
    std::string line;
    std::ifstream myfile ((fname).c_str());
    if (!myfile.is_open()) {
        cout << "Map file " << fname << " does not exist. " << std::endl;
        return false;
    }

    cout << "*** Loading map ***" << std::endl;
    clock_t t = std::clock();
    size_t pos = fname.rfind('.');  // position of the file extension
    map_name = fname.substr(0, pos);  // get the name without extension
    getline (myfile, line);
    boost::char_separator<char> sep(",");
    boost::tokenizer< boost::char_separator<char> > tok(line, sep);
    boost::tokenizer< boost::char_separator<char> >::iterator beg = tok.begin();
    rows = atoi((*beg).c_str());  // read number of rows
    beg++;
    cols = atoi((*beg).c_str());  // read number of cols

    moves[0] = 1;
    moves[1] = -cols;
    moves[2] = -1;
    moves[3] = cols;

    map.resize(cols * rows, 0);

    //DeliverGoal.resize(row*col, false);
    // read map
    //int ep = 0, ag = 0;
    for (int i = 0; i < rows; i++) {
        getline(myfile, line);
        for (int j = 0; j < cols; j++) {
            int id = cols * i + j;
            if (line[j] == '@') // obstacle
                map[id] = 1;
            else  // free space
                map[id] = 0;
        }
    }

    myfile.close();
    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
    cout << "Map size: " << rows << "x" << cols;
    cout << "\tDone! (load time: " << runtime << " s)" << std::endl;
    return true;
}


bool CompetitionSystem::load_agent_tasks(string fname){
	string line;
	std::ifstream myfile(fname.c_str());
	if (!myfile.is_open()) return false;

	getline(myfile, line);
    while (!myfile.eof() && line[0] == '#') {
        getline(myfile, line);
    }

    boost::char_separator<char> sep(",");
    boost::tokenizer<boost::char_separator<char>> tok(line, sep);
    boost::tokenizer<boost::char_separator<char>>::iterator beg = tok.begin();

    num_of_agents = atoi((*beg).c_str());

    // My benchmark
    if (num_of_agents == 0) {
        std::cerr << "The number of agents should be larger than 0" << endl;
        exit(-1);
    }
    starts.resize(num_of_agents);
    task_queue.resize(num_of_agents);
    for (int i = 0; i < num_of_agents; i++) {
        cout << "agent " << i << ": ";

        getline(myfile, line);
        while (!myfile.eof() && line[0] == '#'){
            getline(myfile, line);
        }
        boost::tokenizer<boost::char_separator<char>> tok(line, sep);
        boost::tokenizer<boost::char_separator<char>>::iterator beg = tok.begin();
        // read start [row,col] for agent i
        int num_landmarks = atoi((*beg).c_str());
        beg++;
        auto loc = atoi((*beg).c_str());
        // agent_start_locations[i] = {loc, 0};
        starts[i] = State(loc, 0, 0);
        cout << loc;
        beg++;
        for (int j = 0; j < num_landmarks; j++, beg++) {
            auto loc = atoi((*beg).c_str());
            task_queue[i].push_back(loc);
            cout << " -> " << loc;
        }
        cout << endl;

    }
    myfile.close();

	return true;
}


list<tuple<int, int, int>> CompetitionSystem::move(vector<State>& next_states){
	list<tuple<int, int, int>> finished_tasks; // <agent_id, location, timestep>
    if (valid_moves(curr_states, next_states)){
        curr_states = next_states;
        // check finished tasks;
    }
    // agents do not move
    for (int k = 0; k < num_of_agents; k++) {
        if (!goal_locations[k].empty() && curr_states[k].location == goal_locations[k].front().first){
            goal_locations[k].erase(goal_locations[k].begin());
            finished_tasks.emplace_back(k, curr_states[k].location, timestep + 1);
        }
        paths[k].push_back(curr_states[k]);
    }

    return finished_tasks;
}


// This function might not work correctly with small map (w or h <=2)
bool CompetitionSystem::valid_moves(vector<State>& prev, vector<State> next){
    if (prev.size() != next.size()) {
        return false;
    }

    unordered_map<int, int> occupied;

    for (int i = 0; i < prev.size(); i ++) {
        if (prev[i].location == next[i].location) {
        // check if the rotation is not larger than 90 degree
        // if (abs(prev[i].orientation - next[i].orientation) == 2){
        //   cout << "ERROR: agent " << i << " over-rotates. " << endl;
        //   return false;
        // }
        } else {
            if (prev[i].orientation != next[i].orientation){
                cout << "ERROR: agent " << i << " moves and rotates at the same time. " << endl;
                return false;
            }
            if (next[i].location - prev[i].location != moves[prev[i].orientation]){
                cout << "ERROR: agent " << i << " moves in a wrong direction. " << endl;
                return false;
            }

            if (abs(next[i].location / cols - prev[i].location/cols) + abs(next[i].location % cols - prev[i].location %cols) > 1  ){
                cout << "ERROR: agent " << i << " moves more than 1 steps. " << endl;
                return false;
            }
        }

        if (map[next[i].location] == 1) {
            cout << "ERROR: agent " << i << " moves to an obstacle. " << endl;
            return false;
        }

        if (check_collisions) {
            if (occupied.find(next[i].location) != occupied.end()) {
                cout << "ERROR: agents " << i << " and " << occupied[next[i].location] << " have a vertex conflict. " << endl;
                return false;
            }
            int edge_idx = (prev[i].location + 1) * rows * cols +  next[i].location;
            if (occupied.find(edge_idx) != occupied.end()) {
                cout << "ERROR: agents " << i << " and " << occupied[edge_idx] << " have an edge conflict. " << endl;
                return false;
            }

            occupied[next[i].location] = i;
            int r_edge_idx = (next[i].location + 1) * rows * cols +  prev[i].location;
            occupied[r_edge_idx] = i;
        }
    }

    return true;
}


void CompetitionSystem::sync_shared_env(){
    env->goal_locations = goal_locations;
    env->curr_timestep = timestep;
    env->curr_states = curr_states;
}


void CompetitionSystem::simulate(int simulation_time){
	initialize();
    int num_of_tasks = 0;
    //I just put it out to seperate ours initilize with participants'
    planner->initialize(preprocess_time_limit);
  simulation_time = 20;
	for (; timestep < simulation_time; timestep += 1) {
        cout << "----------------------------" << std::endl;
        cout << "Timestep " << timestep << std::endl;

        // find a plan
        sync_shared_env();
        vector<State> next_states = planner->plan(plan_time_limit);

        // move drives
        list<tuple<int, int, int>> new_finished_tasks = move(next_states);
        cout << new_finished_tasks.size() << " tasks has been finished" << std::endl;
        cout << num_of_tasks << " tasks has been finished by far" << std::endl;

        // update tasks
        for (auto task : new_finished_tasks) {
            int id, loc, t;
            std::tie(id, loc, t) = task;
            finished_tasks[id].emplace_back(loc, t);
            num_of_tasks++;
        }

        update_goal_locations();

        bool complete_all = false;
        for (auto t: goal_locations)
        {
          if(t.empty())
            complete_all = true;
          else
          {
            complete_all = false;
            break;
          }
        }
        if (complete_all)
        {
          cout << std::endl << "All task finished!" << std::endl;
          break;
        }
    }

	cout << std::endl << "Done!" << std::endl;
}


void CompetitionSystem::initialize() {
	// starts.resize(num_of_agents);
	// goal_locations.resize(num_of_agents);
    // task_queue.resize(num_of_drives);

	paths.resize(num_of_agents);
    env->num_of_agents = num_of_agents;
    env->rows = rows;
    env->cols = cols;
    env->map = map;
	finished_tasks.resize(num_of_agents);
	// bool succ = load_records(); // continue simulating from the records
    timestep = 0;
    curr_states = starts;
    goal_locations.resize(num_of_agents);
    // initialize_goal_locations();
    update_goal_locations();

    sync_shared_env();
    //planner->initialize(preprocess_time_limit);
}


void CompetitionSystem::update_goal_locations(){
	for (int k = 0; k < num_of_agents; k++) {
        if (goal_locations[k].empty() && !task_queue[k].empty()) {
            goal_locations[k].emplace_back(task_queue[k].front(), timestep);
            task_queue[k].pop_front();
        }
    }
}

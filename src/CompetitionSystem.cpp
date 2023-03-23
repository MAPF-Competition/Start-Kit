#include <cmath>
#include "CompetitionSystem.h"
#include <boost/tokenizer.hpp>
#include "nlohmann/json.hpp"

using json = nlohmann::ordered_json;

list<Task> BaseSystem::move(vector<Action>& actions){

  for (int k = 0; k < num_of_agents; k++) {
    planner_movements[k].push_back(actions[k]);
  }

  list<Task> finished_tasks_this_timestep; // <agent_id, task_id, timestep>
  if (!valid_moves(curr_states, actions)){
    actions = std::vector<Action>(curr_states.size(), Action::W);
  }

  curr_states = model->result_states(curr_states, actions);
  // agents do not move
  for (int k = 0; k < num_of_agents; k++) {
    if (!assigned_tasks[k].empty() && curr_states[k].location == assigned_tasks[k].front().location){
      Task task = assigned_tasks[k].front();
      assigned_tasks[k].pop_front();
      task.t_completed = timestep;
      finished_tasks_this_timestep.push_back(task);
      events[k].push_back(make_tuple(task.task_id, timestep,"finished"));
    }
    paths[k].push_back(curr_states[k]);
    actual_movements[k].push_back(actions[k]);
  }

  return finished_tasks_this_timestep;
}


// This function might not work correctly with small map (w or h <=2)
bool BaseSystem::valid_moves(vector<State>& prev, vector<Action>& action){
  return model->is_valid(prev, action);
}


void BaseSystem::sync_shared_env(){
  env->goal_locations.resize(num_of_agents);
  for (size_t i = 0; i < num_of_agents; i++){
    env->goal_locations[i].clear();
    for (auto& task: assigned_tasks[i]){
      env->goal_locations[i].push_back({task.location, task.t_assigned });
    }
  }
  env->curr_timestep = timestep;
  env->curr_states = curr_states;
}


void BaseSystem::simulate(int simulation_time){
  initialize();
  int num_of_tasks = 0;
  //I just put it out to seperate ours initilize with participants'
  planner->initialize(preprocess_time_limit);
  for (; timestep < simulation_time; timestep += 1) {
    cout << "----------------------------" << std::endl;
    cout << "Timestep " << timestep << std::endl;

    // find a plan
    sync_shared_env();
    vector<Action> actions = planner->plan(plan_time_limit);

    // move drives
    list<Task> new_finished_tasks = move(actions);
    cout << new_finished_tasks.size() << " tasks has been finished in this timestep" << std::endl;

    // update tasks
    for (auto task : new_finished_tasks) {
      // int id, loc, t;
      // std::tie(id, loc, t) = task;
      finished_tasks[task.agent_assigned].emplace_back(task);
      num_of_tasks++;
    }
    cout << num_of_tasks << " tasks has been finished by far in total" << std::endl;

    update_tasks();

    bool complete_all = false;
    for (auto & t: assigned_tasks)
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


void BaseSystem::initialize() {
  // starts.resize(num_of_agents);
  // goal_locations.resize(num_of_agents);
  // task_queue.resize(num_of_drives);

  paths.resize(num_of_agents);
  events.resize(num_of_agents);
  env->num_of_agents = num_of_agents;
  env->rows = map.rows;
  env->cols = map.cols;
  env->map = map.map;
  finished_tasks.resize(num_of_agents);
  // bool succ = load_records(); // continue simulating from the records
  timestep = 0;
  curr_states = starts;
  assigned_tasks.resize(num_of_agents);
  // initialize_goal_locations();
  update_tasks();

  sync_shared_env();

  actual_movements.resize(num_of_agents);
  planner_movements.resize(num_of_agents);
  for (int i = 0; i < num_of_agents; i++)
    {
      // actual_movements[i].push_back(curr_states[i]);
      // planner_movements[i].push_back(curr_states[i]);
    }
  //planner->initialize(preprocess_time_limit);
}

void BaseSystem::saveErrors(const string &fileName) const
{
  std::ofstream output;
  output.open(fileName, std::ios::out);
  // for (int i = 0; i < num_of_agents; i++)
  //   {
  //     output << "Agent " << i << ": ";
  //     if (option == 0)
  //       {
  //         for (const auto t : actual_movements[i])
  //           // output << "(" << t.location
  //           output << "(" << t.location / map.cols << "," << t.location % map.cols
  //                  << "," << t.orientation << ")->";
  //       }
  //     else if (option == 1)
  //       {
  //         for (const auto t : planner_movements[i])
  //           // output << "(" << t.location
  //           output << "(" << t.location / map.cols << "," << t.location % map.cols
  //                  << "," << t.orientation << ")->";
  //       }
  //     output << endl;
  //   }
  for (auto error: model->errors)
    {
      std::string error_msg;
      int agent1;
      int agent2;
      int timestep;

      std::tie(error_msg,agent1,agent2,timestep) = error;

      output<<"("<<agent1<<","<<agent2<<","<<timestep<<",\""<< error_msg <<"\")"<<endl;
    }
  output.close();
}

void BaseSystem::savePaths(const string &fileName, int option) const
{
  std::ofstream output;
  output.open(fileName, std::ios::out);
  for (int i = 0; i < num_of_agents; i++)
    {
      output << "Agent " << i << ": ";
      if (option == 0)
        {
          bool first = true;
          for (const auto t : actual_movements[i]){
            if (!first){output << ",";} else {
              first = false;
            }
            output << t;
          }
        }
      else if (option == 1)
        {
          bool first = true;
          for (const auto t : planner_movements[i]){
            if (!first){output << ",";} else {
              first = false;
            }
            output << t;
          }
        }
      output << endl;
    }
  output.close();
}

void BaseSystem::saveResults(const string &fileName) const
{
  json js;
  //action model
  js["Action Model"] = "MAPF_T";

  //start locations[x,y,orientation]
  json start = json::array();
  for (int i = 0; i < num_of_agents; i++)
  {
    json s = json::array();
    s.push_back(starts[i].location/map.cols);
    s.push_back(starts[i].location%map.cols);
    switch (starts[i].orientation)
    {
      case 0:
          s.push_back("E");
          break;
      case 1:
        s.push_back("S");
      case 2:
        s.push_back("W");
        break;
      case 3:
        s.push_back("N");
        break;
    }
    start.push_back(s);
  }
  js["Start"] = start;

  //actual paths
  json apaths = json::array();
  for (int i = 0; i < num_of_agents; i++)
  {
    std::string path;
    bool first = true;
    for (const auto action : actual_movements[i])
    {
      if (!first){path+= ",";} else 
      {
        first = false;
      }
      if (action == Action::FW)
      {
        path+="F";
      } 
      else if (action == Action::CR)
      {
        path+="R";
        
      } 
      else if (action == Action::CCR)
      {
        path+="C";
      }
      else
      {
        path+="W";
      }
    }  
    apaths.push_back(path);
  }
  js["Actual Paths"] = apaths;

  //planned paths
  json ppaths = json::array();
  for (int i = 0; i < num_of_agents; i++)
  {
    std::string path;
    bool first = true;
    for (const auto action : planner_movements[i])
    {
      if (!first){path+= ",";} else 
      {
        first = false;
      }
      if (action == Action::FW)
      {
        path+="F";
      } 
      else if (action == Action::CR)
      {
        path+="R";
        
      } 
      else if (action == Action::CCR)
      {
        path+="C";
      }
      else
      {
        path+="W";
      }
    }  
    ppaths.push_back(path);
  }
  js["Planner Paths"] = ppaths;

  //errors
  json errors = json::array();
  for (auto error: model->errors)
  {
    std::string error_msg;
    int agent1;
    int agent2;
    int timestep;
    std::tie(error_msg,agent1,agent2,timestep) = error;
    json e = json::array();
    e.push_back(agent1);
    e.push_back(agent2);
    e.push_back(timestep);
    e.push_back(error_msg);
    errors.push_back(e);

  }
  js["Errors"] = errors;
  
  //events
  json events_json = json::array();
  bool first = true;
  for (int i = 0; i < num_of_agents; i++)
  {
    json event = json::array();
    for(auto e: events[i])
    {
      json ev = json::array();
      std::string event_msg;
      int task_id;
      int timestep;
      std::tie(task_id,timestep,event_msg) = e;
      ev.push_back(task_id);
      if (first)
      {
        ev.push_back(timestep);
        first = false;
      }
      else
      {
        ev.push_back(timestep+1);
      }
      ev.push_back(event_msg);
      event.push_back(ev);
    }
    events_json.push_back(event);
  }
  js["Events"] = events_json;

  //all tasks
  json tasks = json::array();
  for (auto t: all_tasks)
  {
    json task = json::array();
    task.push_back(t.task_id);
    task.push_back(t.location/map.cols);
    task.push_back(t.location%map.cols);
    tasks.push_back(task);
  }
  js["Task Pool"] = tasks;

  std::ofstream f(fileName,std::ios_base::trunc |std::ios_base::out);
  f<<std::setw(4)<<js;

}

// void BaseSystem::saveResults(const string &fileName) const
// {
//   std::ofstream output;
//   output.open(fileName, std::ios::out);
//   output << "{"<<endl;
//   output<<"\"Action Model\":\"MAPF_T\","<<endl;
//   output<<"\"Start\":[";
//   for (int i = 0; i < num_of_agents; i++)
//     {
//       output<<"\"("<<starts[i].location/map.cols<<","<<starts[i].location%map.cols<<","<<starts[i].orientation<<")\"";
//       if(i <num_of_agents-1)
//         output<<",";
//     }
//   output<<"],"<<endl;
//   output << "\"Actual Paths\":"<<endl<<"[";
//   for (int i = 0; i < num_of_agents; i++)
//     {
//       if (i>0)
//         output<<" ";
//       output << "\"";
//       bool first = true;
//       for (const auto t : actual_movements[i]){
//         if (!first){output << ",";} else {
//           first = false;
//         }
//         output << t;
//       }  
//       output<<"\"";
//       if (i < num_of_agents-1)
//         {
//           output<<","<<endl;
//         }
//     }
//   output<<"],"<<endl << "\"Planned Paths\":"<<endl<<"[";
//   for (int i = 0; i < num_of_agents; i++)
//     {
//       if (i>0)
//         output<<" ";
//       output << "\"";
//       bool first = true;
//       for (const auto t : planner_movements[i]){
//         if (!first){output << ",";} else {
//           first = false;
//         }
//         output << t;
//       }
//       output<<"\"";
//       if (i < num_of_agents-1)
//         {
//           output<<","<<endl;
//         }
//     }
//   output<<"],"<<endl<<"\"Errors\":[";
//   int i = 0;
//   for (auto error: model->errors)
//     {
//       std::string error_msg;
//       int agent1;
//       int agent2;
//       int timestep;

//       std::tie(error_msg,agent1,agent2,timestep) = error;

//       output<<"\""<<agent1<<","<<agent2<<","<<timestep<<","<< error_msg <<"\"";
//       if (i < model->errors.size()-1)
//         output<<",";
//       i++;
//     }

//   // TODO I cout task_id:task location here 
//   // This needs to go to the JSON too
//   //
//   // Added comment from Han: I realized that this is problematic because it does not show those unfinished tasks...
//   for (int i = 0; i < num_of_agents; i++)
//     {
//       for(auto & task: finished_tasks[i])
//         {
//           std::cout << task.task_id << ": " << task.location/map.cols << ", " << task.location%map.cols << std::endl;
//         }
//     }


//   output<<"],"<<endl<<"\"Events\":"<<endl<<"["<<endl;
//   for (int i = 0; i < num_of_agents; i++)
//     {
//       // if (events[i].empty())
//       //   continue;
//       //output<<"{"<<endl<<"\"agent\":"<<i<<","<<endl<<"";
//       //output<<"\""<<i<<"\":"<<"{";
//       output<<" [";

//       int j = 0;
//       for(auto e: events[i])
//         {
//           std::string event_msg;
//           int task_id;
//           int timestep;
//           std::tie(task_id,timestep,event_msg) = e;
//           output<<"\"("<<task_id<<","<<timestep<<","<< event_msg <<")\"";
//           if (j < events[i].size()-1)
//             output<<",";
//           j++;
//         }
//       output<<"]";
//       if (i<num_of_agents-1)
//         output<<",";
//       output<<endl;
//     }
//   output<<"]";
//   output<<"}";
//   output.close();
// }

bool FixedAssignSystem::load_agent_tasks(string fname){
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
  int task_id = 0;
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
      task_queue[i].emplace_back(task_id++, loc, 0, i);
      cout << " -> " << loc;
    }
    cout << endl;

  }
  myfile.close();

  return true;
}


void FixedAssignSystem::update_tasks(){
  for (int k = 0; k < num_of_agents; k++) {
    while (assigned_tasks[k].size() < num_tasks_reveal && !task_queue[k].empty()) {
      Task task = task_queue[k].front();
      task_queue[k].pop_front();
      assigned_tasks[k].push_back(task);
      events[k].push_back(make_tuple(task.task_id,timestep,"assigned"));
    }
  }
}



void TaskAssignSystem::update_tasks(){
  for (int k = 0; k < num_of_agents; k++) {
    while (assigned_tasks[k].size() < num_tasks_reveal && !task_queue.empty())
      {
        std::cout << "assigned task " << task_queue.front().task_id <<
          " with loc " << task_queue.front().location << " to agent " << k << std::endl;
        Task task = task_queue.front();
        task.t_assigned = timestep;
        task.agent_assigned = k;
        task_queue.pop_front();
        assigned_tasks[k].push_back(task);
        events[k].push_back(make_tuple(task.task_id,timestep,"assigned"));
      }
  }
}

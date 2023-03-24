# Start-Kit

## Compile the start-kit
Run 
```shell
cmake . -DCMAKE_BUILD_TYPE=Release
make
```

If you plan to use pybind, set `USE_PYBIND` to `TRUE` on Line 6 of `CMakeLists.txt` before running `cmake`.

## Run the start kit

Runing startkit using
command: 
```
./lifelong --inputFolder the_input_folder_location --inputFile the_input_file_name -o output_file_location
```
more info on help



## Input description
+ map_file: the map file location. We support the file format of the MAPF benchmarks.
agent_file: the agent file location. The agent file format is first line agent number, then each line is the location of an agent at timestep 0.
+ task_file: the task file location, task file format similar to agent_file.
num_task_reveal: how many tasks we reveal to the plan at the initial goal assignment or after an agent finish its goal.
+ task_assignment_strategy: how we assign the task to agents, roundrobin (fix assign using round robin strategy) or greedy (next agent gets the next task).

## Output description


+ Action Model: the action model of the current problem, MAPF_T by default.
+ Starts: the start state for each agent, array of arraies, the outer array ordered by agent id, the inner array contains [location x, location y, orientation].
+ Actual Paths: the actual path the system execute, array of strings ordered by agent id. For each action, F--forward, W--wait, C--counter clockwise rotate, R--clockwise rotate.
+ Planner Paths: the path planner gives the system, array of strings ordered by agent id.
Errors: errors from the planner path, array of arrays ordered by timestep, each item in the outer array is an error, the inner array contains [agent 1, agent 2, timestep, error msg].
+ Events: events for each agent, the outer array ordered by agent id represents all the events for a certain agent, for each agent arrary, each event contains[task id, timestep, event msg].
+ Task Pool, an array contains all tasks, each task contains [task id, task location x, task location y].




## Pybinding usage

To write your MAPF planner using Python, you will need to implement your algorithm within the pyMAPFPlanner.py file. In order to use your pyMAPFPlanner with the starter kit, you must set USE_PYBINDING to true in the CMakeLists.txt and then compile it. An example of this process can be found in the test.cpp file within the Python directory.

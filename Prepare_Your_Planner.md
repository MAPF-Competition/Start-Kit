# Prepare Your Entry
To run the program, please refer to [README.md](./README.md) to download the start-kit and compile. 

## Entry Integration

Before you write any code, get familiar with the simulated setups:
- Coordination system: the location of a robot on the map is a tuple (x,y), where x refers to the row the robot is located in, and y refers to the corresponding column. For the first row (the topmost row), x = 0, and for the first column (the leftmost column), y = 0. You can find a visualization [here](./image/coordination_system.pdf)
- Map: the map is a vector of `int`, the index is calculated by linearise the (row, column) of a location to (row * total number of columns of the map) + column, the value is either 1: non-traversable or 0: traversable.
- A `State` of a robot: a state containing the current location (map location index), current timestep and current facing orientation (0:east, 1:south, 2:west, 3:north).
- Tasks of robots: a `task` of a robot contains a list of multiple errands. Each errand is a single location on the map and should be visited one by one in order.
- `Action` enum: the four possible actions are encoded in our start actions as: FW - forward, CR - Clockwise rotate, CCR - Counter clockwise rotate, W - Wait, NA - Unknown actions
- The `Entry` class act as an interface to communicate with the start-kit main simulation. At each timestep, the main simulation will call the compute() function of Entry to get the next task and next schedule for each agent to proceeed. The compute function of the Entry will call the task scheduler to shedule next task for each agent first, call the planenr to plan next actions. 

## What to implement for each track

- Plannning Track:
You need to implement your own planner, whcih will work with default scheduler. Check out ''Implement your planner'' section for more details.

- Scheduling Track:
You need to implement your own scheduler, which will work with default planner. Check out ''Implement your scheduler'' section for more details.

- Combined Track:
You need to implement your own planner and scheduler. You can also modify the entry to meet your needs. Check out Implement your scheduler, Implement your planner and Implement your entry sections for more details.

### Understand the default entry
In `src/Entry.cpp`, you can find the default implementation for entry. In the `Entry::compute()` function, the default entry calls the default scheduler first. After the scheduler finishes, agents might be assigned new tasks and their goals locations need to be updated before the planner is called.
Then, the entry calls the default planner to compute the actions for agents.
The time limit is revealed to both the default scheduler and planner. Inside the default scheduler and planner, you can see each of them using half amount of the time limit.

#### The default scheduler
In `src/TaskScheduler.cpp`, you can find the default task scheduler, which calls functions that are further defined in `default_planner/scheduler.cpp`.
- The preprocessing function of the default scheduler (see `schedule_initialize()` in `scheduler.cpp`) calls the `DefaultPlanner::init_heuristics()` function (see `default_planner/heuristics.cpp`) to initialize a global heuristic table, which will be used to store the distances between different locations. These distances are computed on demand during the simulation. The scheduler uses these distances to estimate the completion time of a given task for a given agent.
- The scheduling function of the default scheduler (see `schedule_plan()` in `scheduler.cpp`) implements a greedy scheduling algorithm: Each time when the `schedule_plan()` function is called, it iterates over all agents. For each agent that does not have an assigned task, the algorithm iterates over tasks that are not assigned to any agent and, among these tasks, assigns to the agent the task with the earliest possible completion time, ignoring conflicts with other agents.

#### The default planner
In `src/MAPFPlanner.cpp`, you can find the default planner implementation, which calls the functions that are furture defined in `default_planner/planner.cpp`. The default planner shares the same heuristic distance tables with the default scheduler. Its `initialize()` function prepares necessary data structures and global heuristic table (if not initialized by scheduler).

The MAPF planner implemented in the default planner is a variant of Traffic Flow Optimised Guided PIBT, [Chen, Z., Harabor, D., Li, J., & Stuckey, P. J. (2024, March). Traffic flow optimisation for lifelong multi-agent path finding. In Proceedings of the AAAI Conference on Artificial Intelligence (Vol. 38, No. 18, pp. 20674-20682).](https://ojs.aaai.org/index.php/AAAI/article/view/30054/31856). The planner first optimises traffic flow assignments for each agent, then computes collision free actions using [Priority Inheritance with Backtracking](https://www.sciencedirect.com/science/article/pii/S0004370222000923) following the optimised traffic flow. A more detailed technical report will be provided soon.

#### Timing parameters for detault planner and default scheduler
The default scheduler and default planner runs in sequencial manner. The default scheduler uses `time_limit/2` as the timelimit to compute schedules.
The default planner uses the remaining time, after scheduler returns, to compute collision-free actions.

You still have some control of the timning behavior of default scheduler and default planner.
File `default_planner/const.h` specifies a few parameters that controls the timing of schduler and planner:
- `PIBT_RUNTIME_PER_100_AGENTS` specifies how many time in ms is requires for PIBT to compute collision free actions per 100 agents. The default planner compute the end time for traffic flow assignment by subtracting PIBT action time from the time limit, so that the remaining time is left for PIBT to return actions.
- `TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE` specifies the traffic flow assignment process end time tolerance in ms. The default planner will end the traffic flow assignment phase this many milliseconds before traffic flow assignment end time.
- `PLANNER_TIMELIMIT_TOLERANCE` The MAPFPlanner will deduct this value from the time limit for default planner.
- `SCHEDULER_TIMELIMIT_TOLERANCE` The TaskScheduler will deduct this value from the time limit for default scheduler.
You are allowed to modify the values of these parameters to reduce/increase the time spend on related components.


### Implement your scheduler
The starting point for implementing your scheduler is to look at the files `src/TaskScheduler.cpp` and `inc/TaskScheduler.h`.
- Implement your own preprocessing function `TaskScheduler::initialize()`. 
- Implement your own scheduling function `TaskScheduler::plan()`. The inputs to the `plan` function are a time limit and a reference to a vector of integers as the result schedule. The ith integer in the result scheduler is the index of the task assigned to the ith agent.
- Don't change the definitions for the `TaskScheduler::initialize()` and `TaskScheduler::plan()` functions. Except this, you are free to add new members/functions to the `TaskScheduler` class.
- Don’t override any operating system-related functions (signal handlers)
- Don’t interfere with the running program -- stack manipulation etc

Start your implementation by understanding the `SharedEnvironment` API. This data structure (defined as `env` in `inc/MAPFPlanner.h`) contains useful information about the simulated setup:
-  num_of_robots: `int`, the total team size.
-  rows: `int`, the number of rows of the map.
-  cols: `int`, the number of columns of the map.
-  map_name: `string`, the map file name.
-  map: vector of `int`, stores the map.  
-  file_storage_path: `string`, used for indicating the path for file storage, refer to section 'Local Preprocessing and Large Files'.
-  goal locations, vector of vector of `pair<int,int>`: current tasks locations allocated to each robot. The first int of a task (pair of int) is the goal location, and the second int indicates the timestep that the task was allocated.
-  current_timestep: `int`, the current timestep according to the simulator. *Please be aware that current_timestep may increment during a `plan()` call. This occurs when a planner exceeds the time limit for a given timestep*
-  curr_states: vector of `State`, the current state for each robot at the current time step.

### Implement your planner

The starting point of your implementation is the file `src/MAPFPlanner.cpp` and `inc/MAPFPlanner.h`. See examples in `src/MAPFPlanner.cpp`
- Implement your preprocessing in the function `MAPFPlanner::initialize()` that is provided to you. 
- Implement your planner in the function `MAPFPlanner::plan()` that provided to you
- Don't change the definitions for the `MAPFPlanner::initialize()` and `MAPFPlanner::plan()` functions. Except this, you are free to add new members/functions to the `MAPFPlanner` class.
- Don’t override any operating system-related functions (signal handlers)
- Don’t interfere with the running program -- stack manipulation etc
 
Similar to the scheduler, the planner can access the `SharedEnvironment` API. You need to use this API for implementing your planner.

### Implement your entry
For participants that compete in the combined track, you can modify the entry freely to meet your needs.
You need to implement your own `Entry::initialize()` and `Entry::compute()` functions and are not allwed to change their definitions. Except this, you are free to add new members/functions to the `Entry` class.
The `Entry::compute()` needs to compute the task schedule and the actions for agents. Although the default entry does this by calling the scheduler and the planner separately, this is not required.

### Unmodifiable files
In any track of the competition, don't modify or interfere any start kit functionalities, including those in following files:
```
src/ActionModel.cpp, src/Evaluation.cpp, src/Logger.cpp, src/States.cpp,src/driver.cpp,
src/CompetitionSystem.cpp, src/Grid.cpp, src/common.cpp, src/TaskManager.cpp, 
inc/ActionModel.h, inc/Evaluation.h, inc/Logger.h, inc/SharedEnv.h, inc/Tasks.h, inc/CompetitionSystem.h, inc/Grid.h,
inc/States.h, inc/common.h, inc/TaskManager.h,
default_planner/Memory.h, default_planner/heap.h, default_planner/pibt.cpp, default_planner/search_node.h, 
default_planner/planner.h, default_planner/search.cpp, default_planner/utils.cpp, default_planner/TrajLNS.h,
default_planner/flow.cpp, default_planner/heuristics.cpp, default_planner/pibt.h, default_planner/scheduler.cpp,
default_planner/search.h, default_planner/utils.h, default_planner/Types.h, default_planner/flow.h,
default_planner/heuristics.h, default_planner/planner.cpp, default_planner/scheduler.h,  
python/common/MAPFbinding.cpp, python/default_planner/pyMAPFPlanner.cpp, 
python/default_scheduler/pyTaskScheduler.hpp, python/user_scheduler/pyTaskScheduler.cpp, python/common/pyEntry.hpp, python/default_planner/pyMAPFPlanner.hpp, python/user_planner/pyMAPFPlanner.cpp, 
python/user_scheduler/pyTaskScheduler.hpp, python/common/pyEnvironment.hpp, 
python/default_scheduler/pyTaskScheduler.cpp, python/user_planner/pyMAPFPlanner.hpp, python/set_track.bash        
```

In planner track, don't modify or interfere any start kit functionalities, including those in following files:
```
inc/TaskScheduler.h, src/TaskScheduler.cpp, inc/Entry.h, src/Entry.cpp
```

In scheduler track, don't modify or interfere any start kit functionalities, including those in following files:
```
inc/MAPFPlanner.h, src/MAPFPlanner.cpp, inc/Entry.h, src/Entry.cpp
```

### Compute command for your robots
- Plan command in the scheduler
to do 

- Plan command in the planner
At every timestep, we will ask your planner to compute the next valid action for each robot subject to a given `time_limit` in ms. The `time_limit` is given as an input parameter to the `compute()` function of `entry.cpp`, which is then passed to `TaskScheduler::plan()` and `MAPFPlanner::plan()`. Note that, for `TaskScheduler::plan()` and `MAPFPlanner::plan()` the start time of current timestep begin at `env->plan_start_time`, indicating the scheduler and the planner should return actions before `env->plan_start_time` plus `time_limit` ms . This is a soft limit, which means if you do not return actions before the `time_limit` elapses, the simulator will continue, and all robots will wait in place until the next planning episode.

At the end of each planning episode, you return one command per robot to the simulator environment. The commands are written into the `actions` vector, which is the input parameter of `plan()` function. The command for robot `i` is a valid `Action` at position `actions[i]` in the vector.

For more details, read the interface implementation in `src/MAPFPlanner.cpp`, `inc/MAPFPlanner.h`, `src/TaskScheduler.cpp`, `src/TaskScheduler.h`, and `inc/SharedEnv.h`.


## Build

Once you implemented your planner, you need to compile your submission for local testing and evaluation.
This section explains how the compilation system works and how to specify dependencies.

### Compile.sh

- The evaluation system will execute the `compile.sh` to build your program on the contest server.
- The evaluation system looks for and executes `./build/lifelong` for evaluation.
- Make sure your `compile.sh` result is an executable called `lifelong` under `build` folder.
- The `compile.sh` build the c++ interface implementation on default. To build python implementation (more on this below), remove the commands after `# build exec for cpp` and uncomment the commands after `# build exec for python`.
- You may adjust the `compile.sh` to match what your implementation needs.

### Dependencies

You are free to use third-party libraries or other dependencies in your implementation. You can do this in several ways:
- Include dependencies in your submission repo,
- Specify dependency packages in apt.txt. These packages must be available for installation through apt-get on Ubuntu 22.

## Python Interface
We also provide a Python interface for Python users based on pybind11.

Dependency: [Pybind11](https://pybind11.readthedocs.io/en/stable/)

The pybind11 bindings are implemented under `python/common`, `python/default_planner`, `python/default_scheduler`, `python/user_planner/`, and `python/user_scheduler/`.
These implementations allow user implemented Python scheduler work with default c++ planner, and allow user implemented Python planner work with default c++ scheduler.
To use the Python interface, simply implement your planner and/or scheduler in:
+ `python/pyMAPFPlanner.py`: this file is where users implement their python-based MAPF palnner algorithms and return actions as a list of actions for each agent.
+ `python/pyTaskScheduler.py`: this file is where users implement their python-based Task Scheduler algorithms and return proposed schedules as a list of task ids for each agent.

### Track Config and Compiling

For each track of the competition, the start-kit uses different combination of python and c++ implementations:
- In Schdduler Track, the start-kit uses `Python scheduler` and `c++ default planner`.
- In Planner Track, the start-kit uses `Python planner` and `c++ default scheduler`.
- In Combined Track, the start-kit uses both `Python planner` and `Python Scheduler`.

When testing your implementation locally, you need to configure the correct track using the `./python/set_track.bash` under the root folder of the start-kit before compiling the code.

For combined track:
```shell
./python/set_track.bash combined
```
For scheduler track:
```shell
./python/set_track.bash scheduler
```
For planner track:
```shell
./python/set_track.bash planner
```

Then edit your compile.sh to make sure it uses only the following content:
```shell
mkdir build
cmake -B build ./ -DCMAKE_BUILD_TYPE=Release -DPYTHON=true
make -C build -j
```

Compile and test your implementation with:
```shell
./compile.sh
./build/lifelong --inputFile ./example_problems/random.domain/random_32_32_20_100.json -o test.json
```
Once compiled, the program looks for `pyMAPFPlanner` python module and `pyTaskScheduler.py` under `./python` or `../python` relative to the current working direction. Additionally, you can specify a path in `config.json` and use cmake flag `-DCOPY_PY_PATH_CONFIG=ON` (which copies the `config.json` to the target build folder), so that the problem will look for the `pyMAPFPlanner` in the specified folder.

Additionally, you can specify a specific python version by `-DPYBIND11_PYTHON_VERSION` or an exact python install by `-DPYTHON_EXECUTABLE`

For example:
```shell
cmake -B build ./ -DCMAKE_BUILD_TYPE=Release -DPYTHON=true -DPYBIND11_PYTHON_VERSION=3.6

#or

cmake -B build ./ -DCMAKE_BUILD_TYPE=Release -DPYTHON=true -DPYTHON_EXECUTABLE=path/to/python
```

Python packages can also be installed through `pip` on the evaluation server, thus you can specify the package you want to install in `pip.txt`.
For example, on default `pip.txt` contains:
```
torch
pybind11-global>=2.10.1
numpy
```

## Evaluation

Once your planner is implemented and compiled, you are ready for local testing and evaluation.
The evaluation system uses official pytorch image [pytorch/pytorch:2.4.1-cuda11.8-cudnn9-devel](https://hub.docker.com/layers/pytorch/pytorch/2.4.1-cuda11.8-cudnn9-devel/images/sha256-ebefd256e8247f1cea8f8cadd77f1944f6c3e65585c4e39a8d4135d29de4a0cb?context=explore) as docker base image to build the evaluation environment, which have GPU driver, cuda, cudnn, and other essential GPU softwares ready. We officially support and tested `Pytorch` in this setup, other frameworks may or may not work in the evaluaiton environment.

### Local Testing
A variety of test problems are provided in the `example_problems` folder. Use any JSON input file there for testing.
Results of the evaluation are placed in a file at `--output_file_location` that you specified as a command line parameter.
For details about the format of input problems and output results refer to the documentation in [Input_Output_Format.md](./Input_Output_Format.md).

### Test in Docker
The evaluation system builds and execuates your implementation in a docker container which acts as a sandbox.
To make sure your implementation builds and runs as expected, you can build the docker container locally.

First, install the latest Docker release on your machine, [https://docs.docker.com/engine/install/](https://docs.docker.com/engine/install/).
Next, build your container using our provided `RunInDocker.sh` script. 
In the remainder of this section, we explain how the script works and how to use your docker container.

#### Using `RunInDocker.sh`

* In the root of your code base, run the command `./RunInDocker.sh`. This script will automatically generate a Dockerfile to build the Docker image based on `pytorch/pytorch:2.4.1-cuda11.8-cudnn9-devel`.
* However, image `pytorch/pytorch:2.4.1-cuda11.8-cudnn9-devel` only support linux/amd64 os/architecture. If you are using another os/architecture (e.g. MacOS with Arm CPUs) and do not use GPU, you could use [ubuntu:jammy](https://hub.docker.com/layers/library/ubuntu/jammy/images/sha256-58148fb210e3d70c972b5e72bdfcd68be667dec91e8a2ed6376b9e9c980cd573?context=explore) as a replacement, by running the script with a base image specified: `./RunInDocker.sh ubuntu:jammy`.
* It will copy your codes to the Docker Environment, install dependencies listed in `apt.txt` using apt-get, and compile your code using `compile.sh`.
* You are inside the docker container when the script finishes.
* You can run the compiled program inside Docker container now.
* The docker image name `<image name>` is `mapf_image` and the container name `<container name>` is `mapf_test`.
* The default working directory is `/MAPF/codes/`.
* You can now test and evaluate your implementation in the container
* Exit the container with the command: `exit`.

#### Start an existing container:
  * In the background: `docker container start <container name>`
  * Interactively: `docker container start -i <container name>`

#### Execute Commands Outside the Container
If the docker container is started in the background, you can run commands from the outside of the docker container (treat the docker container as executable).
 
  * Use prefix: `docker container exec <container name> `for any command you want to execute, for example:
  ```shell
  docker container exec mapf_test ./build/lifelong --inputFile ./example_problems/random.domain/random_20.json -o test.json
  ``` 
 
  * All outputs are stored inside the container. You could copy files from the Docker container. For example: `docker cp mapf_test:/MAPF/codes/test.json ./test.json`, which copies `test.json` to your current working directory.

## Preprocessing and Large File Storage

Prior to the start of each evaluation, we allow your planner 30 minutes of preprocessing time per map to load supporting files and initialise supporting data structures. The `preprocess_time_limit` is specified as a parameter to your planner's `initialize()` function. If your planner's preprocessing operations take longer than `preprocess_time_limit`, your planner fails and the simulation terminates with **exit code 124**. 

Please refer to the documentation in [Working_with_Preprocessed_Data.md](./Working_with_Preprocessed_Data.md) for more details.

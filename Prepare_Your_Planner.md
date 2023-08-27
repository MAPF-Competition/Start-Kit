# Prepare Your Planner
To run the program, please refer to [README.md](./README.md) to download the start-kit and compile. 

## Planner Integration
- Before your planning, getting familiar with the data structures:
    - Coordination system of the map: the location of robot on the map is based on its coordination (x,y), where x refers to the row the robot stays at, and y refers to the column it stays at. For the first row (the topest row), x = 0, and for the first column (the left most column), y = 0.
    - Map: the map is a vector of int, the index is calculated by linearise the (row, column) of a location to (row * total number of columns of the map) + column, the value is either 1: non-traversable or 0: traversable.
    - A State of robot: a state contains the current location (map location index), current timestep and current facing orientation (0:east, 1:south, 2:west, 3:north).
    - Tasks of robots: a task of an robot is a int, represents a single location (linearised) in the map.
    - Action enum: the four possible actions are encoded in our start actions as: FW - forward, CR - Clockwise rotate, CCR - Counter clockwise rotate, W - Wait, NA - Unknown actions
- Start your planing with SharedEnvironment API: Your planner can have access to the shared environment (defined as env in inc/MAPFPlanner.h) and plan based on the things we shared with you in the shared environment. The shared environment contains:
    -  num_of_robots: int, the total team size.
    -  rows: int, the number of rows of the map.
    -  cols: int, the number of columns of the map.
    -  map_name: string, the map file name.
    -  map: vector of int, stores the map.  
    -  file_storage_path: string, use for indicating the path for file storage, refer to section 'Local Preprocessing and Large Files'.
    -  goal locations, vector of vector of pair <int,int>: current tasks locations allocate to each robot.
    -  current_timestep: int, the current time step that our system already simulated the robots' actions.
    -  curr_states: vector of State, the current state for each robot at the current time step, 
- Be aware of time is ticking while planning: at every timestep, we will ask your planner to compute the next valid action for each robot within a given planing time limit. This is given as the input parameter of the function in the planner class (preprocess_time_limit in function 'initialize' and time_limit in function 'plan', both are int). The preprocess_time_limit is the maximum time to preprocess your map. If this process cost more than the time limit, then your planner fails and the system terminates with fail to preprocess within time limit. As for the time_limit in function plan, it is a soft limit, which means if you do not return actions within the time limit, the program will continues, and all robots will wait in place until the next planning episode.
- Return your plan using actions: Once you have plans for the next timestep, you can directly re-assign valuses to the input paramter 'actions' to your plans.
    - actions: vector of Action, given in input parameter. It contains the actions for each robot that we require you to plan for the next timestep. 

- For more details, read the interface implementation in src/MAPFPlanner.cpp, inc/MAPFPlanner.h,  inc/SharedEnv.h.
- Implement your planner in the file src/MAPFPlanner.cpp and inc/MAPFPlanner.h. See examples in src/MAPFPlanner.cpp
    - Implement your preprocessing in the function MAPFPlanner::initialize that provided to you. 
    - Implement your planner in the function MAPFPlanner::plan that provided to you
    - Don’t override any operating system related functions (signal handlers)
    - Don’t modify any start kit functions and modify / call / interfere with any start kit variables or objects, including those in:
      
      src/ActionModel.cpp, src/common.cpp, src/CompetitionSystem.cpp, src/driver.cpp, scr/Evaluation.cpp, src/Grid.cpp, src/Logger.cpp, src/States.cpp, src/Validator.cpp, inc/ActionModel.h, inc/CompetitionSystem.h, Evaluation.h, Grid.h, Logger.h, SharedEnv.h, States.h, Tasks.h, Validator.h, common.h
    - Don’t interfere with the running program -- stack manipulation etc
- Including dependencies in the submission repo,
- or specify your dependency packages in apt.txt. The packages must be available for installation through apt-get on Ubuntu 22.
- Modify compile.sh and make sure your code can be compiled by executing this script.


## Python Interface
We also provide a python interface for python users based on pybind11.

Dependency: [Pybind11](https://pybind11.readthedocs.io/en/stable/)

The pybind11 module mainly contains three files:
+ MAPFBinding.cpp: it binds the C++ classes to the "MAPF" pybind module, allowing users to access  C++ classes such as SharedEnvironment and Action
+ pyMAPFPlanner.py: users can implement their learning-based algorithms and return solutions as a list of actions or a numpy array.
+ pyMAPFPlanner.cpp: pyMAPFPlanner.cpp imports the above Python script and calls relevant Python functions to get the solution and return it to the C++ simulation system

To use the python interface, one can use the following to compile the program that runs pyMAPFPlanner 

```shell
mkdir build
cd buld
cmake ../ -DCMAKE_BUILD_TYPE=Release -DPYTHON=true
make -j
./lifelong --inputFile the_input_file_name -o output_file_location
```
Once compiled, the program looks for `pyMAPFPlanner` python module under `./python` or `../python` relative to the location of `lifelong`. Alternatively, you can specify a path in `config.json` and use cmake flag `-DCOPY_PY_PATH_CONFIG=ON` (which copies the `config.json` to target build folder), so that the problem will look for the `pyMAPFPlanner` in the specified folder.

Additionally, you can specify a specific python version by `-DPYBIND11_PYTHON_VERSION` or an exact python install by `-DPYTHON_EXECUTABLE`

For example:
```shell
cmake ../ -DCMAKE_BUILD_TYPE=Release -DPYTHON=true -DPYBIND11_PYTHON_VERSION=3.6

#or

cmake ../ -DCMAKE_BUILD_TYPE=Release -DPYTHON=true -DPYTHON_EXECUTABLE=path/to/python
```

Python packages can also be installed through apt-get, thus you can specify the package you want to install in `apt.txt`.
For example, to install `numpy`, you can put `python3-numpy` in `apt.txt``.

## Compile.sh

- The evalution system will execuate the `compile.sh` to build your program on the contest server.
- The evalution system looks for `./build/lifelong` for evaluation.
- Make sure your `compile.sh` result an execuatable `lifelong` under `build` folder.
- The compile.sh build the c++ interface implementation on default. To build python implementation remove the commands after `# build exec for cpp` and uncomment the commands after `# build exec for python`.
- You may adjust the `compile.sh` to match what your implementation needs.

## Evaluation Test
- Input Output Description
    - Refer to the [Input_Output_Format.md]
- Local Test
    - Test problems are provided under `example_problems` folder. Use any json input file there for testing.
    - Once your planner is implemented, run your code according to the instructions in Compile and Run section
    - Results are in the output_file_location that you specify in the command line

## Test in Docker
The evaluation system builds and execuates your implementation in a docker container which act as a sandbox.

To make sure your implementation builds and runs in the docker container. You could test your implementation in a docker container locally.

### Install Docker
Install latest docker release on your machine, [https://docs.docker.com/engine/install/](https://docs.docker.com/engine/install/).

### Using `RunInDocker.sh`

* The evaluation system uses a offical [ubuntu:jammy](https://hub.docker.com/layers/library/ubuntu/jammy/images/sha256-b060fffe8e1561c9c3e6dea6db487b900100fc26830b9ea2ec966c151ab4c020?context=explore) image as base image.
* In the root of your code base, run command `./RunInDocker.sh`. This script will automatically generate a Dockerfile to build the docker image.
* It will copy your codes to the Docker Environment, install dependencies listed in `apt.txt` using apt-get, and compile your code using `compile.sh`.
* You are inside the docker container when the script finishes.
* You can run the compiled program in side docker container now.
* The docker image name `<image name>` is `mapf_image` and the container name `<container name>` is `mapf_test`.
* The default working directory is `/MAPF/codes/`.
* You can now test and execuate your implementation in the container
* Exit the container with command: `exit`.

### Start an existing container:
  * In background: `docker container start <container name>`
  * Interactively: `docker container start -i <container name>`

### Execuate Commands Outside the Container
If the docker container is started in background, you can run commands from the outside of the docker container (treat docker container as executable).
 
  * Use prefix: `docker container exec <container name> ` fow any command you want to execuate, for example:
  ```shell
  docker container exec mapf_test ./build/lifelong --inputFile ./example_problems/random.domain/random_20.json -o test.json
  ``` 
 
  * All outputs are stored inside the container. You could copy files from docker container. For example: `docker cp mapf_test:/MAPF/codes/test.json ./test.json`, which copies `test.json` to your current working directory.

## Preprocessing and Large File Storage

### Evaluation Preprocessing
At the start of each benchmark problem evaluation, a preprocessing stage calls `MAPFPlanner:initialize` function:
- The planner has the opportunity to analyse the map and compute auxiliary data before proceeding to the evaluation stage. 
- Preprocessing time is limited to 30 Minutes per map. 
- Nothing you do at this stage will be counted into your final score.
- You have no access to tasks and robot locations at this stage.

### Local Preprocessing and Large Files
As benchmark maps are public accessable and given in the start-kit example_problems. Participants can compute auxiliary data offline or train machine learning models offline and upload the result data for submission evaluation.

Participants can push these auxiliary data to their github submission repo, but file size limits are applied by github. 

To uoload files larger than than 2 GB, participants should use the large file storage service provided by the competition:
- A `Large File Storage` button can be found on the `My Submission` page.
- Click the button opens a storage management panel.
- Upload or delete files using this panel.

The uploaded file will be synced to the evaluation server when evaluation starts:
- The folder stores these files will be mounted to the docker container with read only access.
- The path to the folder can be accessed at `MAPFPlanner::env->file_storage_path`

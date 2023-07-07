# Prepare Your Planner

## Start-kit Download
- Follow the [Submission_Instruction.md] to create an account on the competition website.
- Login with your Github account.
- Find the private repo we created for your account.
- Clone the private repo, which contains a copy of Start-kit codes.

## Compile and Run
- Compile the start kit: 

  Run using command:
```shell
  mkdir build
  cd  build
  cmake ../ -DCMAKE_BUILD_TYPE=Release
  make -j
```
- Run the start kit:
  Once compiled, run using command:
```shell
  ./lifelong --inputFolder the_input_folder_location --inputFile the_input_file_name -o output_file_location
```
  More information can be found on help by using command:
```shell
  ./lifelong --help
```

## Planner Integration
- Implement your planner in the file src/MAPFPlanner.cpp and inc/MAPFPlanner.h. See examples in src/MAPFPlanner.cpp
    - Implement your preprocessing in the function MAPFPlanner::initialize that provided to you. 
    - Implement your planner in the function MAPFPlanner::plan that provided to you
    - Don’t override any operating system related functions (signal handlers)
    - Don’t modify any start kit functions
    - Don’t modify / call / interfere with any start kit variables or objects
    - Don’t interfere with the running program -- stack manipulation etc
- Specify your dependency packages in apt.txt. The packages must be available for installation through apt-get on Ubuntu 22.
- Modify compile.sh and make sure your code can be compiled by executing this script.

We also provide a python interface for python users based on pybind11.

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

## Compile.sh

## Evaluation Test
- Input Output Description
    - Refer to the [Input_Output_Format.md]
- Local Test
    - Test problems are provided under `example_problems` folder. Use any json input file there for testing.
    - Once your planner is implemented, run your code according to the instructions in Compile and Run section
    - Results are in the output_file_location that you specify in the command line

## Visualisation
We provide a visualisattion tool written in python: [https://github.com/MAPF-Competition/MAPF_analysis](https://github.com/MAPF-Competition/MAPF_analysis)

It is able to visualise the output of start-kit program and help participants to debug the implementations.

## Test in Docker

## Preprocessing and Large File Storage

## How to Submit
- Refer to [Submission_Instruction.md](./Submission_Instruction.md)

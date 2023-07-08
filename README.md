# Start-Kit

## Join the competition

Login to the [competition website](http://www.leagueofrobotrunners.org/) with a GitHub account, and we will automatically create a private GitHub submission repo for you.
The repo will be the place that you submit codes to. In the `My Submission` page, you can click "My Repo" to open your GitHub submission repo page.

## Clone your submission repo

Clone your submission repo to your local machine. The repo contains starter code to help you prepare your submission.

```
$ git clone git@github.com:your_submission_repo_address
$ cd your_submission_repo
```

## Compile the start-kit

Using `compile.sh`:
```shell
./compile.sh
```

Using cmake: 
```shell
mkdir build
cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release
make -j
```

## Run the start kit

Runing startkit using command: 
```shell
./lifelong --inputFile the_input_file_name -o output_file_location
```

for example:
```shell
./lifelong --inputFile ../example_problems/random.domain/random_20.json -o test.json
```

more info on help:
```shell
./lifelong --help
```

## Input output description

Please refer to the [Input_Output_Format.md](./Input_Output_Format.md).

## Prepare Your Planner

Please refer to the [Prepare_Your_Planner.md](./Prepare_Your_Planner.md).

## Visualisation
We provide a visualisattion tool written in python: [https://github.com/MAPF-Competition/MAPF_analysis](https://github.com/MAPF-Competition/MAPF_analysis).

It is able to visualise the output of start-kit program and help participants to debug the implementations.

## Submission Instruction

Please refer to the [Submission_Instruction.md](./Submission_Instruction.md).




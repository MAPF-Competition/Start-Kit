# Start-Kit

## Join the competition

Log in to the [competition website](http://www.leagueofrobotrunners.org/) with a GitHub account, and we will automatically create a private GitHub submission repo for you.
The repo will be the place where you submit codes. In the `My Submission` page, you can click "My Repo" to open your GitHub submission repo page.

## Clone your submission repo

Clone your submission repo to your local machine. The repo contains starter codes to help you prepare your submission.

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

Running the start-kit using commands: 
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

## Upgrade Your Start-Kit

If your private start-kit copy repo was created before a start-kit upgrade, you could run the `./upgrade_start_kit.sh` to upgrade your start-kit to the latest version.

You can check `version.txt` to know the current version of your start-kit.

The `upgrade_start_kit.sh` will check which file is marked as an upgrade needed and pull those files from the start-kit.

The upgrade may overwrite some of your changes to `CMakeLists.txt` and `apt.txt`, you could compare the difference using `git diff` and decide whether to revert some modifications on these files.

The upgrade script will not touch any participants' created file, `python/pyMAPFPlanner.py`, `inc/MAPFPlanner.h` and `src/MAPFPlanner.cpp`. So that participants' implementations should not be influenced by the start-kit upgrade.

## Input output description

Please refer to the [Input_Output_Format.md](./Input_Output_Format.md).

## Prepare Your Planner

Please refer to the [Prepare_Your_Planner.md](./Prepare_Your_Planner.md).

## Visualisation
We provide a visualisation tool written in Python: [https://github.com/MAPF-Competition/MAPF_analysis](https://github.com/MAPF-Competition/MAPF_analysis).

It is able to visualise the output of the start-kit program and help participants to debug the implementations.

## Submission Instruction

Please refer to the [Submission_Instruction.md](./Submission_Instruction.md).




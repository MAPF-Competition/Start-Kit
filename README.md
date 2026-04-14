# Start-Kit

## Join the competition

Log in to the [competition website](http://www.leagueofrobotrunners.org/) with a GitHub account, and we will automatically create a private GitHub submission repo for you.
The repo will be the place where you submit code. In the `My Submission` page, you can click "My Repo" to open your GitHub submission repo page.

## Clone your submission repo

Clone your submission repo to your local machine. The repo contains starter codes to help you prepare your submission.

```
$ git clone git@github.com:your_submission_repo_address
$ cd your_submission_repo
```

## Compile the start-kit

### Dependencies

- [cmake >= 3.16](https://cmake.org/)
- [libboost >= 1.49.0](https://www.boost.org/)
- [Python3 >= 3.11](https://www.python.org/)
- [pybind11 >= 3.0.1](https://pybind11.readthedocs.io/en/stable/)

Install dependencies on Ubuntu or Debian Linux:
```shell
sudo apt-get update
sudo apt-get install build-essential libboost-all-dev python3-dev python3-pybind11
```

[Homebrew](https://brew.sh/) is recomanded for installing dependencies on Mac OS.

### Compiling

Using `compile.sh`:
```shell
./compile.sh
```

Using cmake: 
```shell
mkdir build
cmake -B build ./ -DCMAKE_BUILD_TYPE=Release
make -C build -j
```

## Run the start kit

Running the start-kit using commands: 
```shell
./build/lifelong --inputFile the_input_file_name -o output_file_location
```

for example:
```shell
./build/lifelong --inputFile ./example_problems/random.domain/random_32_32_20_100.json -o test.json
```

more info on help:
```shell
./build/lifelong --help
```

## Windows users
If you are a Windows user, the most straightforward method to utilize our start-kits is by employing the WSL (Windows Subsystem for Linux) subsystem. Follow these steps:
1. Install WSL, please refer to [https://learn.microsoft.com/en-us/windows/wsl/install](https://learn.microsoft.com/en-us/windows/wsl/install)
2. Open a shell in WSL and execute the following commands to install the necessary tools (CMake, GCC, Boost, pip, Pybind11):
```shell
sudo apt-get update
sudo apt-get install cmake g++ libboost-all-dev python3-dev python3-pip
pip install pybind11-global numpy
```
3. Employ the commands provided above to compile the start-kit.

While it's technically possible to use our start-kit with Cygwin, Mingw, and MSVC, doing so would be more complex compared to using WSL. You would likely need to configure the environment yourself.

If you are a docker user, another choice is to develop and test your python implementation under a docker environment. You can the re-create the evaluation environment locally on your machine. For more details, check out the [Test in Docker](./Prepare_Your_Submission.md#test-in-docker) section.

## Upgrade Your Start-Kit

If your private start-kit copy repo was created before a start-kit upgrade, always fetch and run the **latest** upgrade script from the official Start-Kit repository (instead of relying on your local copy, which may be outdated).

Run these commands from the root of your private submission repo:

```shell
curl -fsSL -o ./upgrade_start_kit.sh https://raw.githubusercontent.com/MAPF-Competition/Start-Kit/main/upgrade_start_kit.sh
bash ./upgrade_start_kit.sh
```

Alternative (wget):

```shell
wget -qO ./upgrade_start_kit.sh https://raw.githubusercontent.com/MAPF-Competition/Start-Kit/main/upgrade_start_kit.sh
bash ./upgrade_start_kit.sh
```

By default, this runs in **dry-run** mode and prints the upgrade plan only.

Apply the upgrade:

```shell
bash ./upgrade_start_kit.sh --apply
```

Useful options:
- `--to-version 3.1.0` to target a specific Start-Kit release (default: latest release).
- `--allow-dirty` to apply on a non-clean git tree (not recommended).
- `--source-branch dev` to test upgrades from an upstream branch (for example unpublished changes on `dev`).

Example for testing branch updates:

```shell
bash ./upgrade_start_kit.sh --source-branch dev --apply
```

You can check `version.txt` to know the current version of your start-kit.

The upgrader is manifest-driven and release-by-release. It restores managed/protected files from official Start-Kit release tags, stages those updates, and does not commit automatically.

For participant-modifiable files that changed between your local version and the target release, the upgrader performs a 3-way merge (local/base/remote). If conflicts exist, they are left as standard git conflict markers for manual resolution.

The upgrade script will not touch most participant implementation files.

## Input output description

Please refer to the [Input_Output_Format.md](./Input_Output_Format.md).

## Prepare Your Planner

Please refer to the [Prepare_Your_Submission.md](./Prepare_Your_Submission.md).

## Debug and Visualise Your Planner
We provide a visualisation tool written in Python: [https://github.com/MAPF-Competition/PlanViz](https://github.com/MAPF-Competition/PlanViz).
It is able to visualise the output of the start-kit program and help participants debug the implementations. 

Please refer to the project website for more information. Also the document [Debug_and_Visualise_Your_Planner](./Debug_and_Visualise_Your_Planner.md) which provides helpful hints for interpreting and diagnosing planner output.

## Submission Instruction

Please refer to the [Submission_Instruction.md](./Submission_Instruction.md).




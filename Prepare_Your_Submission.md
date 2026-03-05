# Prepare Your Entry

To run the program, please refer to [README.md](./README.md) to download the start-kit and compile.

## System Overview

![System Overview](./image/system_overview.png)

The image shows the system overview of the start-kit. At each *planning episode*:

1. The competition system calls the `Entry` to produce a **proposed schedule** and a **proposed plan**, providing a `SharedEnvironment` (`SharedEnv`) that stores the current robot states, tasks, and schedule.
   - Internally, the default `Entry` calls `TaskScheduler` to compute the proposed schedule, and `MAPFPlanner` to compute the proposed plan.
2. Once `Entry` returns, the proposed plan is passed to the **Executor**, which converts/merges it into per-agent **staged actions** (a small “committed” window of actions).
3. The **Simulator** validates and executes actions at a finer *execution tick* resolution (see “Robot dynamics & time scales” below), and updates the resulting system state.
4. The Task Manager validates the proposed schedule, checks task progress/completions, and reveals new tasks.
5. The updated state, schedule, and revealed tasks are passed back to `Entry` through `SharedEnv` at the next planning episode.

---

## Important concepts

Before you start, get familiar with the following concepts in the code base:

- **Coordinate system**: the location of a robot on the map is a tuple `(row, col)`, where `row` refers to the row the robot is located in, and `col` refers to the corresponding column.
  - For the first row (the topmost row), `row = 0`, and for the first column (the leftmost column), `col = 0`.
  - You can find a visualization here: [coordination_system.pdf](./image/coordination_system.pdf)

- **Map**: the map is a vector of `int`. The index is calculated by linearizing the `(row, col)` coordinate into:
  - `loc = row * (number_of_columns) + col`
  - `map[loc] = 1` means **non-traversable**, `map[loc] = 0` means **traversable**.

- **`State`** (defined in `inc/States.h`):
  - `location`: the linearized map location index
  - `timestep`: the **execution tick** timestep (not the planning-episode counter)
  - `orientation`: `0:east, 1:south, 2:west, 3:north`
  - `counter`: a `Counter` struct tracking progress within the current action (see below)
  - `delay`: a `Delay` struct (used by the simulator to model execution delays)
  - `moveType`: one of `{Transition, Rotation, None}` indicating whether the agent is currently “in-between” cells (during a forward move) or in a rotation phase.

- **`Counter`** (defined in `inc/Counter.h`):
  - `count`: how many execution ticks have elapsed in the current action
  - `maxCount`: how many execution ticks are required to complete one action
  - The simulator increments `count` every execution tick; when `count` reaches `maxCount`, it resets to `0` and the discrete move/rotation is completed.

- **`Delay`** (defined in `inc/Delay.h`):
  - `inDelay`: when `true`, the simulator forces the agent to **wait** at execution time.
  - (You typically don’t need to set this in your planner; treat it as part of the system dynamics.)

- **`Task`** (defined in `inc/Tasks.h`):
  - A task contains a list of multiple errands, stored in `locations`.
  - `task_id`: the task id
  - `agent_assigned`: the id of assigned robot (if any)
  - `idx_next_loc`: the index of the next unfinished errand in `locations`
  - Each errand is a single location on the map and must be visited one-by-one in order.
  - A task can be reassigned to a different robot if its first errand has not been completed (i.e. `idx_next_loc == 0`).
    However once a robot **opens** its assigned task (i.e., completes the first errand), the task cannot be reassigned to other robots.

- **`Action` enum** (defined in `inc/ActionModel.h`):
  - `FW`: forward
  - `CR`: clockwise rotate
  - `CCR`: counter-clockwise rotate
  - `W`: wait
  - `NA`: not applicable (reserved by the framework; do not use it as a normal control action)

- **`Plan`** (defined in `inc/Plan.h`):
  - `Plan` contains `actions`, a 2D array:
    - `plan.actions[agent_id][k]` is the **k-th planned planner-level action** for that agent.
  - This plan is processed by the Executor into `staged_actions` (a limited window of actions that can be executed before the next plan can reliably take effect).

---

## Robot dynamics & time scales (planning vs execution)

This start-kit runs with **two time scales**:

1. **Planning episodes**: when `Entry::compute()` is called, subject to `time_limit` (ms).
2. **Execution ticks**: the simulator advances in small ticks of length `env->action_time` (ms).  
   Each planner-level action (FW/CR/CCR/W) takes `env->max_counter` execution ticks to complete.

So the time to finish *one* planner-level action is approximately:

> `action_duration_ms = env->action_time * env->max_counter`

The Executor uses the minimum planner communication time to decide how much of a returned plan can be meaningfully staged/executed before the next plan arrives:

> `window_size ≈ env->min_planner_communication_time / (env->action_time * env->max_counter)`

**What this means for you (planner track):**
- You can return a multi-step plan, but only the *front part* (up to a window) may be staged/executed immediately.
- The simulator checks safety during intermediate transition/rotation states too (because states include `moveType` + `counter`).

---

## SharedEnv

The `SharedEnvironment` API provides the necessary information for you to compute schedules and plans.
This data structure (defined as `env` in `inc/MAPFPlanner.h`, `inc/Entry.h`, `inc/TaskScheduler.h`, and `inc/SharedEnv.h`) describes the simulated setup and the state of the current planning episode:

### Map / problem setup
- `num_of_agents`: `int`, the total team size.
- `rows`: `int`, the number of rows of the map.
- `cols`: `int`, the number of columns of the map.
- `map_name`: `string`, the map file name.
- `map`: `vector<int>`, stores the map (1 = obstacle, 0 = free).
- `file_storage_path`: `string`, used for indicating the path for file storage, refer to section “Preprocessing and Large File Storage”.

### Tasks / scheduling
- `task_pool`: `unordered_map<int, Task>`, all revealed but unfinished tasks, keyed by `task_id`.
- `new_tasks`: `vector<int>`, the `task_id` of newly revealed tasks at the current planning episode.
- `new_freeagents`: `vector<int>`, ids of robots that just became free (completed their tasks) at the current planning episode.
- `curr_task_schedule`: `vector<int>`, the current schedule returned by the Task Manager.  
  `curr_task_schedule[i]` is the `task_id` scheduled for robot `i`, `-1` means no scheduled task.
- `goal_locations`: `vector<vector<pair<int,int>>>`, current goal location(s) for each robot, aligned with scheduled tasks:
  - `goal_locations[i][0].first` is typically the first unfinished errand location
  - `goal_locations[i][0].second` indicates when that goal was allocated (timestep of allocation)

### Time / synchronisation
- `plan_start_time`: `chrono::steady_clock::time_point`, stores the exact time `Entry::compute()` was called.
  Your `Entry::compute()` should return within `time_limit` ms after this timestamp.
- `curr_timestep`: `int`, a planning-episode counter (used by the framework).
- `system_timestep`: `int`, the **execution tick** timestep (advanced by the simulator every execution tick).

### Robot states (important!)
- `curr_states`: `vector<State>`, the **planning/synchronised** state for each robot at the start of the planning episode.
  - In the current framework, this corresponds to the simulator’s predicted state after processing the latest staged plan window.
- `start_states`: `vector<State>`, the planning-episode “start” state snapshot (useful for debugging/warm-starting).
- `system_states`: `vector<State>`, the **live execution** state at the current execution tick.

### Execution interface
- `staged_actions`: `vector<vector<Action>>`, the currently staged (queued) actions for each agent that are committed for execution.
  - Most participants should treat this as read-only; it can be useful for warm-starting or debugging.

### Timing parameters (execution / communication)
- `min_planner_communication_time`: `int` (ms), minimum communication time between planner updates and robots.
- `action_time`: `int` (ms), the duration of one execution tick.
- `max_counter`: `int`, number of execution ticks required to complete one planner-level action.

---

## Entry Integration

### Understand the default entry

In `src/Entry.cpp`, you can find the default implementation for `Entry`.

In `Entry::compute()`:
1. The default entry calls the scheduler first.
2. After the scheduler finishes, robots might be assigned new tasks and their goal locations (next errand of the scheduled task) are stored in `env->goal_locations` for planner reference.
3. Then, the entry calls the planner to compute a **multi-step plan** (a `Plan` object).

Inside the default scheduler and planner, you can see each of them using roughly half of the provided time budget.

### The default scheduler

In `src/TaskScheduler.cpp`, you can find the default task scheduler, which calls functions defined in `default_planner/scheduler.cpp`.

- The preprocessing function (see `schedule_initialize()` in `scheduler.cpp`) calls `DefaultPlanner::init_heuristics()` to initialize a global heuristic table, used to store distances between locations (computed on demand during the simulation).
- The scheduling function (see `schedule_plan()` in `scheduler.cpp`) implements a greedy scheduling algorithm:
  each time `schedule_plan()` is called, it assigns an unassigned task to each free robot based on estimated completion time.

### The default planner

In `src/MAPFPlanner.cpp`, you can find the default planner implementation, which calls the functions defined in `default_planner/planner.cpp`.

The MAPF planner implemented in the default planner is a variant of Traffic Flow Optimised Guided PIBT:
- Chen, Z., Harabor, D., Li, J., & Stuckey, P. J. (2024). *Traffic flow optimisation for lifelong multi-agent path finding.* AAAI.

The planner first optimises traffic flow assignments for each robot, then computes collision-free actions using PIBT following the optimised traffic flow.

> ⚠️ **NOTE** ⚠️ The default planner is an anytime algorithm: the time it has to compute a plan directly impacts plan quality.

Better plans and better schedules can both substantially influence performance on the leaderboard.
How to allocate time between these components is an important part of a successful strategy.

---

## What to implement for each track

- **Scheduling Track**: implement your own scheduler (works with the default planner).  
  See: [Implement your scheduler](./Prepare_Your_Submission.md#implement-your-scheduler)

- **Planning Track**: implement your own planner (works with the default scheduler).  
  See: [Implement your planner](./Prepare_Your_Submission.md#implement-your-planner)

- **Combined Track**: implement your own planner and scheduler; you may also modify the entry.  
  See: [Implement your planner](./Prepare_Your_Submission.md#implement-your-planner),  
  [Implement your scheduler](./Prepare_Your_Submission.md#implement-your-scheduler),  
  [Implement your entry](./Prepare_Your_Submission.md#implement-your-entry)

---

## Implement your scheduler

If you are only competing in the scheduler track, you can ignore “Implement your planner” and “Implement your entry”.
Instead, read [Entry Integration](./Prepare_Your_Submission.md#entry-integration) to understand how the default planner and default Entry work.

The starting point for implementing your scheduler is:
- `src/TaskScheduler.cpp`
- `inc/TaskScheduler.h`

You need to implement:
- `TaskScheduler::initialize(int preprocess_time_limit)`
- `TaskScheduler::plan(int time_limit, std::vector<int> & proposed_schedule)`

Notes:
- Don’t change the function definitions for `initialize()` and `plan()`.  
  (You may add new members/functions to the `TaskScheduler` class.)
- Don’t override OS-related functions (signal handlers).
- Don’t interfere with the running program (stack manipulation, etc.).

At each planning episode, the scheduler can access:
- `env->task_pool` to see which tasks are available
- `env->curr_task_schedule` to see the current schedule
- `env->curr_states` / `env->system_states` if you need state information for assignment

The scheduler should return one task id per robot in `proposed_schedule`:
- `proposed_schedule[i]` is the `task_id` assigned to robot `i`
- `-1` means no assigned task

A schedule is invalid if:
- one task is assigned to more than one agent,
- it includes completed tasks,
- a task already opened by an agent is reassigned to another agent,
- an opened task is dropped by assigning `-1` to that agent.

If the scheduler returns an invalid schedule, it will be rejected and the current schedule remains unchanged.

---

## Implement your planner

If you are only competing in the planner track, you can ignore “Implement your scheduler” and “Implement your entry”.
Instead, read [Entry Integration](./Prepare_Your_Submission.md#entry-integration) to understand how the default scheduler and default Entry work.

The starting point is:
- `src/MAPFPlanner.cpp`
- `inc/MAPFPlanner.h`

You need to implement:
- `MAPFPlanner::initialize(int preprocess_time_limit)`
- `MAPFPlanner::plan(int time_limit, Plan & plan)`

Notes:
- Don’t change the function definitions for `initialize()` and `plan()`.  
  (You may add new members/functions to the `MAPFPlanner` class.)
- Don’t override OS-related functions (signal handlers).
- Don’t interfere with the running program (stack manipulation, etc.).

### What you return

At the end of each planning episode, fill `plan.actions`:

- `plan.actions` must be a `vector<vector<Action>>`
- `plan.actions.size()` should be `env->num_of_agents`
- `plan.actions[i]` is the action sequence for agent `i`
- Prefer making the plan “rectangular” (all agents have the same horizon length)

The Executor will stage/execute a limited window of actions based on:
- `env->min_planner_communication_time`
- `env->action_time`
- `env->max_counter`

### Validity / safety

Your plan must avoid:
- moving into obstacles,
- vertex conflicts,
- edge conflicts,
- and (importantly) conflicts that could occur during intermediate transition/rotation phases, since actions are executed over multiple execution ticks.

If the planner returns invalid actions (or times out), the simulator will fall back to safe behavior (waits), and errors will be recorded in the log.

The planner can access:
- `env->goal_locations` for next errand locations of assigned tasks
- `env->curr_task_schedule` and `env->task_pool` for task details
- `env->curr_states` (planning/synchronised state) and `env->system_states` (live execution state)

---

## Implement your entry

For combined track participants, you may modify `Entry`, `MAPFPlanner`, and `TaskScheduler` to meet your needs.

You must implement:
- `Entry::initialize(int preprocess_time_limit)`
- `Entry::compute(int time_limit, Plan & plan, std::vector<int> & proposed_schedule)`

You are not allowed to change these function signatures.  
Except for this, you are free to add new members/functions to the `Entry` class.

The `Entry::compute()` needs to compute both the task schedule and the plan.
The default entry does this by calling scheduler and planner separately, but that is not required.

---

## Timing parameters for default planner and scheduler

At every planning episode, the system calls `Entry::compute(time_limit, ...)`.

- The `time_limit` is in milliseconds.
- The timing window begins at `env->plan_start_time`.
- This is a *soft* limit: if you exceed it, the simulator continues and will apply safe fallback behaviors.

The default scheduler and default planner run sequentially:
- scheduler uses roughly `time_limit/2`
- planner uses the remaining time

You can control timing behavior in `default_planner/const.h`:
- `PIBT_RUNTIME_PER_100_AGENTS`
- `TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE`
- `PLANNER_TIMELIMIT_TOLERANCE`
- `SCHEDULER_TIMELIMIT_TOLERANCE`

You are allowed to modify these parameter values.

---

## Unmodifiable files

Except for the related implementation files for each track and some modifiable files stated in the following sections, most of the starter kit files are unmodifiable, and you must ensure that their functionalities are not interfered with.

Please refer to [Evaluation_Environment.md](./Evaluation_Environment.md) for more details.

---

## Build

Once you implement your planner, you need to compile your submission for local testing and evaluation.

This section explains how the compilation system works and how to specify dependencies.

### compile.sh

- The evaluation system will execute `compile.sh` to build your program on the contest server.
- The evaluation system looks for and executes `./build/lifelong` for evaluation.
- Make sure your `compile.sh` produces an executable called `lifelong` under the `build/` folder.
- The default `compile.sh` builds the C++ interface implementation.

To build the Python implementation (more on this below), remove the commands after `# build exec for cpp` and uncomment the commands after `# build exec for python`.

You may adjust `compile.sh` to match what your implementation needs. You are allowed to customize `compile.sh` and `CMakeLists.txt`, but you must ensure that start-kit functionalities are not interfered with and that all related features are compiled (especially those implemented in unmodifiable files).

### Dependencies

You are free to use third-party libraries or other dependencies in your implementation. You can do this in several ways:

- Include dependencies in your submission repo
- Specify dependency packages in `apt.txt` (must be available via `apt-get` on Ubuntu 22.04)
- Specify Python packages in `pip.txt`

---

## Python Interface

We also provide a Python interface for Python users based on pybind11.

Dependency:
- [pybind11](https://pybind11.readthedocs.io/en/stable/)

The pybind11 bindings are implemented under:
- `python/common`
- `python/default_planner`
- `python/default_scheduler`
- `python/user_planner/`
- `python/user_scheduler/`

### Where to implement (Python)

- `python/pyMAPFPlanner.py`:
  - implement your Python planner and return a **2D action plan** (`List[List[Action]]`) aligned with `Plan.actions`
- `python/pyTaskScheduler.py`:
  - implement your Python scheduler and return `List[int]` task ids (one per agent)

### Track config and compiling

For each track, the start-kit uses a different combination of Python and C++ implementations:

- Scheduler Track: **Python scheduler** + **C++ default planner**
- Planner Track: **Python planner** + **C++ default scheduler**
- Combined Track: **Python planner** + **Python scheduler**

When testing locally, configure the correct track using:

```shell
./python/set_track.bash combined
./python/set_track.bash scheduler
./python/set_track.bash planner
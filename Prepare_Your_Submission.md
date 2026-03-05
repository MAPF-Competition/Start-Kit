# Prepare Your Entry

To run the program, please refer to [README.md](./README.md) to download the start-kit and compile.

This document explains:
- how the competition system calls `Entry`, `Planner`, `Scheduler`, and the **Executor**,
- what data you receive in `SharedEnvironment`,
- what you are expected to return (schedule + **multi-step plan**),
- how timing / timeouts work in the **two-rate** (planner vs executor) loop,
- how to build and test locally.

---

## System Overview

![system_overview](./image/sequence_diagram.png)

The start-kit runs a **two-rate control loop**:

### A) Planning update (slow loop)
Periodically (every multiple execution ticks), the competition system:
1. syncs the latest environment info into `SharedEnvironment` (`env`),
2. calls `Entry::compute(time_limit, proposed_plan, proposed_schedule)`.

Inside `Entry::compute()` (default implementation in `src/Entry.cpp`):
- `TaskScheduler::plan()` produces `proposed_schedule`,
- `Entry::update_goal_locations()` updates `env->goal_locations`,
- `MAPFPlanner::plan()` produces `proposed_plan` (a **multi-step plan**, see below).

After `Entry::compute()` returns, the system:
3. calls the **Executor** to process the new plan and update per-agent `staged_actions` (the action queues that will be executed until the next plan update),
4. continues execution using those staged actions.

### B) Execution tick (fast loop)
Every execution tick (even while planning is running), the system:
1. calls the Executor to decide a per-agent execution command (GO/STOP),
2. converts GO/STOP + staged actions into **requested actions** for this tick,
3. applies delays (may force some agents to STOP/Wait),
4. simulates one tick of motion with the action model (continuous overlap-based collision handling),
5. updates robot states and task progress.

---

## Important concepts (code-level)

Before you start, get familiar with the following concepts in the code base:

### Coordinate system
The location of a robot on the map is (row, col).
- row increases downward, starting from 0 at the top
- col increases rightward, starting from 0 at the left

See [coordination_system.pdf](./image/coordination_system.pdf).

### Map representation
The map is a `vector<int>` in row-major order.
Index of (row, col) is `row * cols + col`.
Cell value:
- `1` = obstacle (non-traversable)
- `0` = free (traversable)

### `State` (robot state)
Defined in `inc/States.h`.

Besides `(location, timestep, orientation)`, the state includes:
- `counter`: progress within the current action (`inc/Counter.h`)
- `delay`: whether the agent is currently delayed (`inc/Delay.h`)
- `moveType`: whether the agent is mid-transition or mid-rotation (`Transition / Rotation / None`)

This matters because actions may take **multiple ticks** to finish.

### `Task`
Defined in `inc/Tasks.h`.

A task is a sequence of errands (`locations`) that must be visited in order.
Fields include:
- `task_id`
- `agent_assigned`
- `idx_next_loc` (index of next unfinished errand)

A task can be reassigned only if its first errand is not completed (`idx_next_loc == 0`).
Once a robot opens a task (completes the first errand), it cannot be reassigned.

### Planner-level `Action`
Defined in `inc/ActionModel.h`:
- `FW`  (Forward)
- `CR`  (Clockwise rotate)
- `CCR` (Counter-clockwise rotate)
- `W`   (Wait)
- `NA`  (Not applicable / reserved; do not output in your plan)

**Planner actions are grid-level intent** (e.g., `FW` means “move toward the next cell”).
The planner does **not** output fractional movements.

### Execution-level `ExecutionCommand`
Also defined in `inc/ActionModel.h`:
- `GO`   (allow progress this tick)
- `STOP` (pause this tick)

The executor outputs `GO/STOP` every tick; the simulator then applies the action model and safety rules.

### `Plan` and `staged_actions`
- `Plan` is defined in `inc/Plan.h`.
- A plan stores `plan.actions`, typed as `vector<vector<int>>`.

Interpretation:
- `plan.actions[i]` is a sequence of planner-level actions for agent `i`,
  encoded as integers corresponding to the `Action` enum (`FW=0, CR=1, CCR=2, W=3, ...`).

The executor converts the returned `Plan` into `staged_actions`:
- `staged_actions[i]` is a queue of actions that are ready to be executed for agent `i`.
- These staged actions are executed tick-by-tick until a new plan is adopted.

You can read the current staged actions in `env->staged_actions`.

---

## SharedEnvironment (`SharedEnv`)

`SharedEnvironment` is defined in `inc/SharedEnv.h` and is available as `env`.

It contains:

### Static environment info
- `num_of_agents`, `rows`, `cols`
- `map_name`
- `map` (vector<int>)
- `file_storage_path` (for storing/loading auxiliary data)

### Time / execution parameters
- `plan_start_time`: timepoint when the system called `Entry::compute()`
- `min_planner_communication_time`: minimum time between plan updates (ms)
- `action_time`: time budget per execution tick (ms)
- `max_counter`: number of ticks needed to complete one action (FW/CR/CCR)

### Robot state views
This system exposes two useful “views” of robot states:

- `system_states`: the **current physical states** in the executor loop (tick-by-tick).
- `curr_states`: the **planning snapshot states** used for planning updates.
  These may be predicted/processed states (e.g., after staging a plan window),
  and may differ from `system_states`.

Also available:
- `system_timestep`: the current execution tick
- `curr_timestep`: the timestep visible to the planner (may advance during planning)
- `staged_actions`: current per-agent action queues staged for execution

### Task info
- `curr_task_schedule`: current schedule (agent -> task_id, or -1)
- `task_pool`: revealed but unfinished tasks (task_id -> Task)
- `new_tasks`: newly revealed tasks at this update
- `new_freeagents`: agents that just finished a task at this update
- `goal_locations`: per-agent next goal (derived from schedule; updated by Entry)

> Note: `curr_timestep` may advance during `Entry::compute()` because the executor continues to tick while planning is running. Do not assume time is frozen during planning.

---

## Entry Integration

### Understand the default Entry
Default Entry is in `src/Entry.cpp`.

`Entry::compute(time_limit, plan, proposed_schedule)` runs:
1) scheduler: `TaskScheduler::plan(time_limit, proposed_schedule)`
2) goal update: `Entry::update_goal_locations(proposed_schedule)`
3) planner: `MAPFPlanner::plan(time_limit, plan)`

The scheduler and planner share the same `time_limit` and are called sequentially.
Both should budget time using `env->plan_start_time`.

---

## Planner / Scheduler / Executor APIs (fixed)

These function signatures must not be changed:

### Scheduler
Files: `inc/TaskScheduler.h`, `src/TaskScheduler.cpp`
- `TaskScheduler::initialize(int preprocess_time_limit)`
- `TaskScheduler::plan(int time_limit, vector<int>& proposed_schedule)`

Return:
- `proposed_schedule[i] = task_id` or `-1` (no task)

Invalid schedule examples:
- same task assigned to multiple agents
- assigning completed/unrevealed/nonexistent tasks
- reassigning an already opened task to another agent
- assigning `-1` to an agent that already opened a task (invalid)

If a schedule is invalid or times out, the system keeps the previous valid schedule.

### Planner
Files: `inc/MAPFPlanner.h`, `src/MAPFPlanner.cpp`
- `MAPFPlanner::initialize(int preprocess_time_limit)`
- `MAPFPlanner::plan(int time_limit, Plan& plan)`

Return:
- `plan.actions[i]` = sequence of actions for agent `i` (multi-step plan)

**Multi-step plans are supported and recommended.**
If an agent runs out of staged actions, it will be STOPped until new actions are staged.

### Executor
Files: `inc/Executor.h`, `src/Executor.cpp`
- `Executor::initialize(int preprocess_time_limit)`
- `Executor::process_new_plan(int sync_time_limit, Plan& plan, vector<vector<Action>>& staged_actions)`
- `Executor::next_command(int exec_time_limit, vector<ExecutionCommand>& agent_command)`

Return:
- `process_new_plan(...)` updates `staged_actions` and returns predicted states.
- `next_command(...)` outputs `GO/STOP` per agent for the next tick.

The default executor provides a baseline policy and a default plan-staging strategy.

---

## How plans are received and executed (important)

### Plan staging window
The executor may choose to stage only part of the returned plan.
The default executor uses a “window” idea:
- it stages enough actions to cover roughly one planner communication interval.

This prevents staging far into the future when a new plan may arrive soon.

### When is a staged action consumed?
In the simulator, a staged action is removed from the front of the queue only when it is completed:
- `W` is consumed after one tick of waiting
- `FW` is consumed when the agent reaches the next cell center (location changes)
- `CR/CCR` is consumed when the orientation update completes (orientation changes)

If an agent is STOPped mid-action (or delayed), the action remains at the front and resumes later.

---

## Execution model (Counters, delays, and overlap-based safety)

### Multi-tick actions (counter)
`FW`, `CR`, and `CCR` take multiple execution ticks.
Progress is tracked by `State.counter`.
When the counter completes, the discrete location/orientation updates.

### Delays
During a delay tick, an agent is forced to Wait/STOP.
This pauses counter progress and may create congestion.

### Collisions and safety (no vertex/edge rules)
This branch does not use “vertex” or “edge” conflicts.
Instead, safety is enforced by geometric overlap:
- agents are modeled as axis-aligned squares (“safety bubbles”)
- obstacles are solid squares
- overlap is checked continuously over a tick (swept collision)

If an agent’s requested motion is unsafe, the action model may override it to `W` for that tick.
This override can happen per-agent (not necessarily “everyone waits”).

Practical implication:
- Your planner can output collision-free plans at the grid level, but the executor/action model
  is the final authority and may still STOP/WAIT agents due to continuous overlap constraints,
  delays, and dependency resolution.

---

## Timing and timeouts (what really happens)

There are several time budgets in the system:

### Preprocessing (hard limit)
`Entry::initialize(preprocess_time_limit)` is called before evaluation begins.
If preprocessing exceeds this limit, the run terminates (exit code 124).

### Planning update time limit (soft limit)
`Entry::compute(time_limit, ...)` is time-bounded.
If it runs late:
- the executor continues ticking using the last accepted staged actions,
- the next plan update is delayed until the current planning call finishes.

In other words: late planning reduces how often your plan can be updated; the simulation does not freeze.

### Executor per-tick time limit
`Executor::next_command(exec_time_limit)` is called every execution tick.
If it is slow, you effectively lose execution time (the simulation may advance using safe waiting behavior to compensate).
Keep `next_command` lightweight.

### Plan processing time limit
`Executor::process_new_plan(sync_time_limit, ...)` should also be efficient.
If it is slow, the system compensates similarly by advancing time with safe waiting behavior.

---

## What to implement for each track

(If your competition year uses different track naming, follow the website instructions; the APIs above remain fixed.)

- Scheduling Track: implement your scheduler; default planner + default executor run with it.
- Planning Track: implement your planner; default scheduler + default executor run with it.
- Execution Track (if enabled): implement your executor; default planner/scheduler provide intent.
- Combined Track: implement planner + scheduler (and optionally executor if allowed/required).

You can also modify `Entry` in combined settings, but do not change API signatures.

---

## Build

Once you implement your components, compile your submission for local testing and evaluation.

### compile.sh
- The evaluation system will execute `compile.sh` on the contest server.
- The evaluation system looks for and executes `./build/lifelong`.
- Make sure `compile.sh` produces an executable named `lifelong` under the `build/` folder.

You may customize `compile.sh` and `CMakeLists.txt`, but you must not break start-kit functionality, especially unmodifiable files (see [Evaluation_Environment.md](./Evaluation_Environment.md)).

### Dependencies
You can use third-party libraries:
- vendored in your repo, or
- installed via `apt.txt` (Ubuntu 22 packages available via apt-get)

---

## Python Interface (pybind11)

We provide Python bindings via pybind11 for planner/scheduler development.

Track selection:
- Use `./python/set_track.bash` to assemble binding sources into `./python/tmp/` before compiling.
  - `./python/set_track.bash combined`
  - `./python/set_track.bash planner`
  - `./python/set_track.bash scheduler`

Then compile with PYTHON enabled:
- set `-DPYTHON=true` in cmake (see `compile.sh` notes).

Python packages:
- add to `pip.txt` if needed (packages are installed during evaluation).

> Note: because the executor and multi-step plan staging are handled in C++,
> Python planner/scheduler users typically only implement planning/scheduling logic.
> If you need deeper control over execution policy, use the C++ executor.

---

## Evaluation

### Local testing
Use any JSON input file in `example_problems/`:
- `./build/lifelong --inputFile ./example_problems/... -o test.json`

For input/output details see [Input_Output_Format.md](./Input_Output_Format.md).

### Test in Docker
Use `./RunInDocker.sh` to reproduce the evaluation environment locally.
See the existing instructions in this repo for running commands inside/outside the container.

---

## Preprocessing and Large File Storage

Before evaluation, your entry has preprocessing time per map to load supporting files and initialize data structures.
If preprocessing exceeds the limit, the run terminates (exit code 124).

See [Working_with_Preprocessed_Data.md](./Working_with_Preprocessed_Data.md) for how to cache and load auxiliary data safely.

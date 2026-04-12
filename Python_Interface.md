# Python Interface

The start-kit provides a **Python interface** built on [pybind11](https://pybind11.readthedocs.io/).
You can implement your planner, scheduler, and/or executor in Python while the rest of the system (simulation engine, action model, collision handling) stays in C++.

**Any combination** of Python and C++ components is supported — for example, a Python scheduler with the default C++ planner and executor, or all three in Python.

## Quick Start

```bash
# Compile (pybind11 is a required dependency; install via pip or your package manager)
./compile.sh

# Run with all-Python components:
./build/lifelong -i example_problems/random.domain/random_32_32_20_100.json \
    --plannerPython true --schedulerPython true --executorPython true \
    -s 1000 -t 500

# Mix Python and C++ (e.g. Python scheduler, C++ planner + executor):
./build/lifelong -i example_problems/random.domain/random_32_32_20_100.json \
    --schedulerPython true \
    -s 1000 -t 500
```

The three CLI flags default to `false`. Set any of them to `true` to use the corresponding Python implementation.

## File Layout

```
python/
  common/                # Bridge layer (do not modify)
    MAPFbinding.cpp      # pybind11 bindings for MAPF C++ types
    opaque_types.h       # Zero-copy container type declarations
    pyMAPFPlanner.hpp/cpp
    pyTaskScheduler.hpp/cpp
    pyExecutor.hpp/cpp
    pyEntry.hpp          # Orchestrator that swaps C++/Python components
  user_planner/
    pyMAPFPlanner.py     # ← Your planner implementation goes here
  user_scheduler/
    pyTaskScheduler.py   # ← Your scheduler implementation goes here
  user_executor/
    pyExecutor.py        # ← Your executor implementation goes here
```

You only need to edit the three files under `user_planner/`, `user_scheduler/`, and `user_executor/`.

## Python APIs

All three Python classes receive `self.env`, a **zero-copy reference** to the C++ `SharedEnvironment` object. Every field documented in the [SharedEnvironment section](./Prepare_Your_Submission.md#sharedenvironment-sharedenv) is directly accessible from Python (e.g., `self.env.curr_states`, `self.env.task_pool`, `self.env.map`).

The `MAPF` module is automatically available via `import MAPF`. It exposes all C++ types: `MAPF.Action` (FW, CR, CCR, W, NA), `MAPF.ExecutionCommand` (GO, STOP), `MAPF.State`, `MAPF.Task`, `MAPF.Plan`, `MAPF.SharedEnvironment`, and all container types.

### Planner (`python/user_planner/pyMAPFPlanner.py`)

```python
import MAPF

class pyMAPFPlanner:
    def __init__(self):
        self.env = None  # Set by the bridge before initialize()

    def initialize(self, preprocess_time_limit: int):
        """Called once before the simulation starts."""
        pass

    def plan(self, time_limit: int) -> list:
        """
        Called periodically to compute a multi-step plan.

        Returns:
            list[list[MAPF.Action]]: actions[agent_id][timestep]
        """
        return [[MAPF.Action.W] for _ in range(self.env.num_of_agents)]
```

### Scheduler (`python/user_scheduler/pyTaskScheduler.py`)

```python
import MAPF

class pyTaskScheduler:
    def __init__(self):
        self.env = None

    def initialize(self, preprocess_time_limit: int):
        pass

    def plan(self, time_limit: int) -> list:
        """
        Called periodically to produce a task schedule.

        Returns:
            list[int]: proposed_schedule[agent_id] = task_id or -1
        """
        return list(self.env.curr_task_schedule)
```

### Executor (`python/user_executor/pyExecutor.py`)

```python
import MAPF

class pyExecutor:
    def __init__(self):
        self.env = None

    def initialize(self, preprocess_time_limit: int):
        pass

    def process_new_plan(self, sync_time_limit: int, plan, staged_actions) -> list:
        """
        Called when a new plan arrives from the planner.
        `plan` is a MAPF.Plan (reference). `staged_actions` is a mutable
        reference to per-agent action queues — modify it IN-PLACE.

        Returns:
            list[MAPF.State]: predicted state per agent after staging.
        """
        # Example: stage all planned actions
        for i in range(len(plan.actions)):
            for a in plan.actions[i]:
                staged_actions[i].append(a)
        return list(self.env.curr_states)

    def next_command(self, exec_time_limit: int) -> list:
        """
        Called every execution tick.

        Returns:
            list[MAPF.ExecutionCommand]: GO or STOP per agent.
        """
        return [MAPF.ExecutionCommand.GO for _ in range(self.env.num_of_agents)]
```

## Key Details

**Zero-copy data access.**
Container types (`VectorInt`, `VectorState`, `VectorAction`, `VectorVectorAction`, `TaskPool`, etc.) are exposed as opaque bindings. Accessing `self.env.map`, `self.env.curr_states`, or `self.env.staged_actions` does **not** copy the underlying C++ data — you read and write the same memory the C++ simulation uses.

**Modifying `staged_actions` in-place.**
In `process_new_plan`, the `staged_actions` parameter is a mutable reference to the C++ `vector<vector<Action>>`. Use `.append()` to add actions and `del staged_actions[i][start:end]` to remove them. Do not reassign the variable itself (e.g., `staged_actions = [...]` will not work).

**Accessing `goal_locations` and `task_pool`.**
`goal_locations[i]` is a vector of `pair<int,int>`. Use the helper functions `MAPF.pair_first(p)` and `MAPF.pair_second(p)` to access the first/second elements.
`task_pool` is a dict-like object keyed by `task_id`: iterate with `for task_id in self.env.task_pool`, access with `self.env.task_pool[task_id]`.

**Class and method names are fixed.**
The bridge loads classes named exactly `pyMAPFPlanner`, `pyTaskScheduler`, and `pyExecutor` from their respective files. Do not rename the classes or method signatures.

**Timing.**
The same timing rules apply to Python as to C++: `initialize` must finish within `preprocess_time_limit`, `plan`/`process_new_plan`/`next_command` should respect their time budgets. Keep `next_command` especially lightweight — it is called every execution tick.

**Python dependencies.**
List any Python packages in `pip.txt` (one per line). They will be installed via `pip install` in the evaluation environment. The standard library and `numpy` are always available.

## `compile.sh` for Python

The default `compile.sh` builds a single `lifelong` executable that embeds the Python interpreter. No special build changes are needed — the same binary supports both C++ and Python components, selected at runtime via CLI flags.

```bash
# compile.sh (default content — works for both C++ and Python)
#!/bin/bash
mkdir -p build
cmake -B build ./ -DCMAKE_BUILD_TYPE=Release
make -C build -j
```

## MAPF Module Reference

The `MAPF` module exposes the following types:

| Python type | C++ type | Notes |
|:---|:---|:---|
| `MAPF.Action` | `Action` enum | `.FW`, `.CR`, `.CCR`, `.W`, `.NA` |
| `MAPF.ExecutionCommand` | `ExecutionCommand` enum | `.GO`, `.STOP` |
| `MAPF.State` | `State` | Fields: `.location`, `.orientation`, `.timestep`, `.counter`, `.delay` |
| `MAPF.Task` | `Task` | Fields: `.task_id`, `.locations`, `.idx_next_loc`, `.agent_assigned`, `.is_finished()` |
| `MAPF.Plan` | `Plan` | Field: `.actions` (VectorVectorAction) |
| `MAPF.SharedEnvironment` | `SharedEnvironment` | All fields from `SharedEnv.h` |
| `MAPF.Counter` | `Counter` | Fields: `.count`, `.maxCount` |
| `MAPF.Delay` | `Delay` | Fields: `.minDelay`, `.maxDelay`, `.inDelay` |
| `MAPF.PairInt` | `pair<int,int>` | Use `MAPF.pair_first(p)` / `MAPF.pair_second(p)` |
| `MAPF.VectorInt` | `vector<int>` | List-like, supports `len()`, indexing, iteration |
| `MAPF.VectorState` | `vector<State>` | List-like |
| `MAPF.VectorAction` | `vector<Action>` | List-like, supports `.append()` |
| `MAPF.VectorVectorAction` | `vector<vector<Action>>` | Nested list-like |
| `MAPF.TaskPool` | `unordered_map<int, Task>` | Dict-like, supports `in`, `[]`, iteration |

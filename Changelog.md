# Changelog
Version 2.1.1 - 2024-11-18
----------------------------
Fixed:
- Bug: Solution costs counted multiple times.
- Bug: When reassigning a task to an agent with a smaller id, `task->agent_assigned` will be reset to -1 instead of the smaller id agent.  

Added:
- Docs explaining the anytime behaviour of the default planner.


Version 2.1.0 - 2024-11-15
----------------------------
Added:
- Added `new_tasks` API to `SharedEnvironment` to notify the entry of new tasks released at the current timestep.
- Added `new_freeagents` API to `SharedEnvironment` to notify the entry of new free robots, who newly completed their assigned task, at the current timestep.
- Added `--logDetailLevel` option to specify the level of details of the log file.

Changes:
- API `vector<Task> task_pool` in `SharedEnvironment` is now removed. Use `unordered_map<int, Task> task_pool` instead, which uses `task_id` as the key.
- Documentation updated to reflect the changes in the API.
- The competition system now waits for entry to return and records the number of timeouts, then progress to the simulator. This prevents the entry using the unrecorded time spent on the simulator.
- Output JSON records a number of entry timeouts, invalid schedules, and invalid actions.
- Default Scheduler now uses the new API to schedule tasks.
- The `update_goal_locations` function in Default Entry is updated to use the new `task_pool` API. (Warning, when updating your entry, make sure you review the changes on `Entry.cpp` and decide how you adapt the changes to your entry implementation.)
- Update the Python binding to support the updated API.
- Updated the example python scheduler to use the new API. (Warning, when updating, make sure you review the changes on `pyTaskScheduler.py` and decide how you adapt the changes to your scheduler implementation.)
- Assigning `task_id` `-1` to an agent to indicate no assigned task. This drops any existing but unopened task. However, assign `-1` to an agent with opened task leads to an invalid schedule error

Version 2.0.0 - 2024-10-2
----------------------------

Added:
- Added support for task scheduler
- Added default planner and task scheduler implementation
- Added support for combined track
- Added Python support for new tracks
- Added Evaluation_Environment.md with details of the evaluation environment

Changes:
- Time limit uses `ms` as the unit
- Input-output format of the starter kit updated for the 2024 competition, with support for tasks with precedence constraints
- Updated documentation for new features
- Updated RunInDocker.sh to recreate the evaluation environment used for the 2024 competition
- Evaluation environment support PyTorch and CUDA
- rename Prepare_Your_Planner.md to Prepare_Your_Submission.md


Version 1.1.5 - 2023-11-21
----------------------------
Fixed:
- An emergency fix on program crash if a log file is not set with CLI.

Version 1.1.4 - 2023-11-18
----------------------------
Fixed:
- Fixed a bug causing segmentation fault on preprocess timeout. Preprocess timeout will now terminate the program with exit code 124.
- Fixed a bug causing segmentation fault on reading env->goal_locations when the plan timelimit exceeded.

Changes:
- Updated documentation to explain the preprocess timeout behaviour.
- Updated documentation to better explain online timeout behaviour. In particular, we spell out that env->curr_timestep may increment during the `plan()` call if it exceeds the given plan timelimit.

Version 1.1.3 - 2023-10-24
----------------------------
Added:
- Add Working_with_Preprocessed_Data.md to explain how you can work with your preprocessed data.
- Add Debug_and_Visualiser_Your_Planner.md to explain how you can use the JSON output to debug and visualise with PlanViz
  
Changed:
- Additional option `OutputScreen` in the input argument, which allows you to choose the level of details of the output JSON file.
- Readme, Parepare_Your_Planner, and compile.sh suggests running the start-kit under repo root directory.
- Simplified duplicated output appears in couts and log files
- Updated documentation (add more descriptions regarding the coordination system in Prepare_Your_Planner.md)
- Updated documentation (add corresponding descriptions of `OutputScreen` in Input_Output_Format.md)
- Terminate all processes when the main process (the simulation) is terminated.

Fixed:
- Fixed issue with running start-kit under repo root directory python cannot find compiled MAPF module for importing.
- Fixed issue with missing fie_storage_path for pybind.

Version 1.1.2 - 2023-08-29
----------------------------
Fixed:
- Parsing 'T' symbol commit missing in the git history.


Version 1.1.1 - 2023-08-27
----------------------------
Added:
- More example problems for each map

Changed:
- Updated documentation (adding more descriptions regarding map symbols and task assignment strategy): Input_Output_Format.md
- Updated documentation (adding more instructions for windows users): README.md
- Updated documentation (adding more descriptions regarding coordination system of the map): Prepare_Your_Planner.md
- Remove unused class file in start kit: Validator.h Validator.cpp

Fixed:
- Fixed issue with example task file for start and goal locations are not connected.
- Fixed issue with detecting map symbol 'T'.
- Fixed issue with agents validation of out of the map boundary.

Version 1.1.0 - 2023-08-04
----------------------------
Added:
- Add changelog.md for tracking version changes
- More example problems for warehouse domain

Changed:
- Updated documentation (adding more descriptions): Input_Output_Format.md, Prepare_Your_Planner.md 
- Add upgrade instructions in README.md
- Add warnings in the logger when the number of tasks in the problem file is smaller than the team size
- Merge py_driver.cpp and drive.cpp into one
- Simplified the CMakeList for Python binding, allowing specifying Python version/executable with cmake flags.
- Add both ./python and ../python to the Python interpreter path. Allow custom path with config.json.

Fixed:
- Fixed issue with example task file for warehouse large only has 1000 tasks (smaller than team size), of which 10000 tasks must be replaced
- Fixed issue with edge conflict checking causing integer overflow bug.

Version 1.0.0 - 2023-07-13
----------------------------
Initial release of the project

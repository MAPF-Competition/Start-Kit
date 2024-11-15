
## Input Arguments

| options               |                                                                                                                                        |
|-----------------------|----------------------------------------------------------------------------------------------------------------------------------------|
| --help                |                                                                                                                                        |
| --inputFile           | String <br /> The input file describes the problem to be solved in JSON format                                                                                                    |
| --output              | String  <br /> output results from the evaluation into a JSON formated file. If no file specified, the default name is 'output.json'                                                                                                   |
| --outputScreen        | Int (=1)  <br /> The level of details in the output file (=1 to show all, = 2 to show problem summary, start and actual path and =3 to show only problem summary).                                                                                                |
| --simulationTime      | Int <br /> The maximum number of timesteps allowed for solving the problem, we sometimes refer to this as planning horizon                                                                                             |
| --fileStoragePath              | String  <br /> the large file storage path. If this parameter is not specified, the program will read from environment variable `$LORR_LARGE_FILE_STORAGE_PATH`. If specified, we ignore the environment variable.                                                                                                |
| --planTimeLimit       | Int <br /> The amount of time available for planning in each timestep (in millseconds), if this time limit is exceeded by the planner, all robots are issued wait commands at current timestep  and the task schedule keeps the same as last time. Note that for 2023 round the planTimeLimit is planning time for each timestep in seconds, so if you are using the planner from 2023 participants, please carefully checkout the timeout settings in the planner.                                                                                 |
| --preprocessTimeLimit | Int <br /> The amount of time in millseconds available for loading and precomputing auxiliary data before the problem-solving process begins                                                                                              |
| --logFile             | String  <br /> An output file that records all warnings and errors issued by the simulator in the event of invalid or incomplete actions from the planner                                                                                                     |
| --logDetailLevel      | Int (=1) <br /> The minimum severity level of log messages to display, 1--showing all the messages, 2--showing warnings and fatal errors, 3--showing fatal errors only |


## Input Problem File (in JSON format)

All paths here is the relative path relative to the location of input JSON file

| properties             |                                                                                                                                                                                                                                                |
|------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| mapFile                | String <br /> The relative path to the file that describes the grid environment input. We use the grid map format as described in the next section (with a section link to the relevant section)                                                                                                                                                                  |
| agentFile              | String <br /> The relative path to the file that describes the start locations for robots. The first line indicates the number of robots n. The following n lines correspond to the start locations of the n robots..* |
| taskFile               | String <br /> The relative path to the file that describes the locations for tasks. The first line indicates the number of tasks m. The following m lines contain multiple integers that each corresponds to a location of the task on the grid,\* and the order of locations on the same line specifies the order of locations that should be completed for this task.               |
| teamSize               | Int <br /> The number of robots in the simulation                                                                                                                                                                                                       |
| numTasksReveal         | Float <br /> The multiplier of tasks revealed in the task pool. We always keep numTasksReveal times teamSize of tasks revealed in the task pool. If in one timestep, k tasks are finished, then the system will add k tasks into the task pool.                                                                                                                                                                                        |

\* We linearize the a 2-D coordinate and use a single integer to represent a location. Given a location (row,column) and the map height (total number of rows) and width (total number of columns), the linearized location = row*width+column.

## Map File Format

All maps begin with the lines:

> type octile <br />
> height y <br />
> width x <br />
> map <br />

Map Symbols:
| symbols                |                                                                                                                                                                                                                                                |
|------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| @                      | hard obstacle.                                                                                                                                                                  |
| T                      | hard obstacle (for 'trees' in game environment).                                                                                                                                                                  |
| .                      | free space |
| E                      | emitter point (for ‘delivery’ goal) - traversable              |
| S                      | service point (for ‘pick up’ goal) - traversable                                                                                                                                                                                                        |


## Output File (in JSON format)

The output file of `./lifelong` is a JSON file consisting of the planner output, actual paths of robots, and the statistics.

The following table defines the properties that appear in the output file.

| properties      |                                                                                                                                                                                                                                                                                                                                                                                 |
|-----------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| actionModel     | String <br /> The name of the action model used for the robots in the simulator, this value is always "MAPF_T" which indicates MAPF with Turnings (i.e. robots can be orientated in any of the 4 cardinal directions, the avaliable actions are forward, clockwise turn, counter-clockwise turn, and wait)                                                                                                                                                                                                                                                                                                  |
| teamSize        | Int <br /> The number of robots in the simulation                                                                                                                                                                                                                                                                                                                                                     |
| start           | List <br />A list of start locations. The length of the list is the number of robots.                                                                                                                                                                                                                                                                                           |
| numTaskFinished | Int <br />Number of finished tasks.                                                                                                                                                                                                                                                                                                                                             |
| sumOfCost       | Int <br />                                                                                                                                                                                                                                                                                                                                                                      |
| makespan        | Int <br />                                                                                                                                                                                                                                                                                                                                                                 |
| actualPaths     | List <br /> A list of n strings, where n is the number of robots. Each string represents a sequence of action symbols, separated by commas (“,”).<br />  Action symbols: <br />+ “F”  Forward <br />+ “R” Clockwise rotate <br /> + “C” Counter-clockwise rotate <br />+ “W” Wait <br /> + “T” Implicit wait (only in plannerPaths and corresponding to planner timeout or missing actions) |
| plannerPaths    | List<br /> A list of n strings, where n is the number of robots. Each string represents a sequence of action symbols, separated by commas.                                                                                                                                                                                                                                      |
| plannerTimes    | List <br />A list of compute times in seconds during each planning episode of the entry.                                                                                                                                                                                                                                                                          |
| errors          | List <br />A list of action errors. Each error is represented by a list [task_id, robot1, robot2, timestep, description] where robot1, robot2, and timestep are integers and description is a string. robot1 and robot2 correspond to the id of robots that are involved in the error (robot2=-1 in case there is only one robot involved). The description is a message for the error.  |
| actualSchedule | List <br /> A list of n strings, where n is the number of robots. Each string represents a new/renewed (valid) first task schedule and the time it changes, which is separated by ",". For each task schedule, the time and task schedule are separated by ":" and the task schedule, which contains the first task of the agent, i.e., "0:0,5:3," means for an agent, task 0 is scheduled as the first task of the agent at timestep 0 and task 3 is scheduled as the first task of the agent at timestep 5|
| plannerSchedule | List <br /> A list of n strings, where n is the number of robots. Each string represents a new/renewed (valid or invalid) task schedule the taskScheduler propsoed and the time it changes, which is separated by ",". |
| events          | List <br />A list of (task) events. Each event is represented by a list [timestep, agent_id, task_id, sequence_id], they are all integers.  The sequence_id indicates the progress of the task and which errand in the task the agent is moving towards, also indicating how many errands in this task are finished.                                                                                                                  |
| scheduleErrors          | List <br />A list of schedule errors. Each error is represented by a list [task_id, robot1, robot2, timestep, description] where task_id robot1, robot2, and timestep are integers and description is a string. robot1 and robot2 correspond to the id of robots that are involved in the error (robot2=-1 in case there is only one robot involved). The description is a message for the error.  |
| tasks           | List <br />A list of tasks. Each task is represented by a list of multiple integers representing each location of the task [id, release_time, errand_sequence], errand_sequence is a list of errand locations: [x1,y1,x2,y2, ... ... ]|
| numPlannerErrors | Int <br /> The number of planner errors (invalid actions). |
| numScheduleErrors | Int <br /> The number of schedule errors (invalid schedules). |
| numEntryTimeouts | Int <br /> The number of entry timeouts. |





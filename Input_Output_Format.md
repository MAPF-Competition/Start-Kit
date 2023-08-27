
## Input Arguments

| options               |                                                                                                                                        |
|-----------------------|----------------------------------------------------------------------------------------------------------------------------------------|
| --help                |                                                                                                                                        |
| --inputFile           | String <br /> The input file describes the problem to be solved in JSON format                                                                                                    |
| --output              | String  <br /> The output file describes the planner result in JSON format                                                                                                   |
| --evaluationMode      | Boolean  <br /> when run in this mode the program will check the validity of the planned and executed actions in an existing plan, as specified by the --output parameter |
| --simulationTime      | Int <br /> The maximum number of timesteps allowed for solving the problem, we sometimes refer to this as planning horizon                                                                                             |
| --planTimeLimit       | Int <br /> The amount of time available for planning in each timestep, if this time limit exceeded by the planner, all robots are issued wait commands at current timestep                                                                                       |
| --preprocessTimeLimit | Int <br /> The amount of time in seconds available for loading and precomputing auxiliary data before the problem-solving process begins                                                                                              |
| --logFile             | String  <br /> An output file that records all warnings and errors issued by the simulator in the event of invalid or incomplete actions from the planner                                                                                                     |

## Input Problem File (in JSON format)

All paths here is the relative path relative to the location of input JSON file

| properties             |                                                                                                                                                                                                                                                |
|------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| mapFile                | String <br /> The relative path to the file that describes the grid environment input. We use the grid map format as described in the next section (with a section link to the relevant section)                                                                                                                                                                  |
| robotFile              | String <br /> The relative path to the file that describes the start locations for robots. The first line indicates the number of robots n. The following n lines correspond to the start locations of the n robots..* |
| taskFile               | String <br /> The relative path to the file that describes the locations for tasks. The first line indicates the number of tasks m. The following m lines contain single integers that correspond to the locations of the m tasks on the grid.\*               |
| numTasksReveal         | Int (=1) <br /> The number of tasks/errands revealed to an robot at any one time. Every time an robot finishes a task/errand a new task is revealed. By default, this is 1, which means only the next 1 task/errand is known to the robot.                                                                                                                                                                                       |
| taskAssignmentStrategy | String (=roundrobin) <br /> The strategy for assigning tasks (`greedy`,`roundrobin` or `roundrobin-fixed`). Every time an robot finished a task/errand, a task assignment strategy decides the next task assigned to the robot.  `greedy` will assign the next unassgined task from task file line by line. While for `roundrobin` and `roundrobin-fixed`, tasks assignment is pre-deceided by the system using a round robin strategy, where the $i_{th}$ robot will get the $i + n_i*(team size)$ task and $n_i$ is the task counter for robot $i$, starts from 0 and growth by 1 upon a task finish. The difference between "roundrobin" and "roundrobin-fixed" is that for "roundrobin-fixed", the simulation will stop when all the tasks from the task files are finished, and for "roundrobin" the task assignment system will iteratively read the task file from the first line to simulate planning for "infinite" tasks/errands.                                                                                                                                                                   |
| teamSize               | Int <br /> The number of robots in the simulation                                                                                                                                                                                                       |


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


## Onput File (in JSON format)

The output file of `./lifelong` is a JSON file consisting of the planner output, actual paths of robots, and the statistics.

The following table defines the properties that appear in the output file.

| properties      |                                                                                                                                                                                                                                                                                                                                                                                 |
|-----------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| actionModel     | String <br /> The name of the action model used for the robots in the simulator, this value is always "MAPF_T" which indicates MAPF with Turnings (i.e. robots can be orientated in any of the 4 cardinal directions, the avaliable actions are forward, clockwise turn, counter-clockwise turn, and wait)                                                                                                                                                                                                                                                                                                  |
| AllValid        | String <br /> A string either "Yes" or "No" that describes all the actions returned by your planner are valid actions or not                                                                                                                                                                                                                                                                                                                                                     |
| teamSize        | Int <br /> The number of robots in the simulation                                                                                                                                                                                                                                                                                                                                                     |
| start           | List <br />A list of start locations. The length of the list is the number of robots.                                                                                                                                                                                                                                                                                           |
| numTaskFinished | Int <br />Number of finished tasks.                                                                                                                                                                                                                                                                                                                                             |
| sumOfCost       | Int <br />                                                                                                                                                                                                                                                                                                                                                                      |
| makespan        | Int <br />                                                                                                                                                                                                                                                                                                                                                                 |
| actualPaths     | List <br /> A list of n strings, where n is the number of robots. Each string represents a sequence of action symbols, separated by commas (“,”).<br />  Action symbols: <br />+ “F”  Forward <br />+ “R” Clockwise rotate <br /> + “C” Counter-clockwise rotate <br />+ “W” Wait <br /> + “T” Implicit wait (only in plannerPaths and corresponding to planner timeout or missing actions) |
| plannerPaths    | List<br /> A list of n strings, where n is the number of robots. Each string represents a sequence of action symbols, separated by commas.                                                                                                                                                                                                                                      |
| plannerTimes    | List <br />A list of planner times in seconds during each planning episode.                                                                                                                                                                                                                                                                          |
| errors          | List <br />A list of errors. Each error is represented by a list [robot1, robot2, timestep, description] where robot1, robot2, and timestep are integers and description is a string. robot1 and robot2 correspond to the id of robots that are involved in the error (robot2=-1 in case there is only one robot involved). The description is a message for the error.         |
| events          | List <br />A list of (task) events. Each event is represented by a list [timestep, id, description] where timestep and id are integers, and description is a string. id corresponds to a task id in tasks. The description can be “assigned” or “finished”.                                                                                                                     |
| tasks           | List <br />A list of tasks. Each task is represented by a list of three integers [id, source, target].                                                                                                                                                                                                                                                                          |


## Input Arguments

| options               |                                                                                                                                        |
|-----------------------|----------------------------------------------------------------------------------------------------------------------------------------|
| --help                |                                                                                                                                        |
| --inputFile           | String <br /> The input JSON file                                                                                                      |
| --output              | String  <br /> The output JSON file                                                                                                    |
| --evaluationMode      | Boolean  <br /> If set true, the start kit will evaluate the planner outputs in the output JSON file instead of running the simulation |
| --simulationTime      | Int <br /> The #iterations for simulation                                                                                              |
| --planTimeLimit       | Int <br /> The time limit for planner in second                                                                                        |
| --preprocessTimeLimit | Int <br /> The #iterations for simulation                                                                                              |
| --logFile             | String  <br /> Issue log file name                                                                                                     |

## Input JSON Format


| properties             |                                                                                                                                                                                                                                                |
|------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| mapFile                | String <br /> File that describes the graph. We support the benchmark format.                                                                                                                                                                  |
| agentFile              | String <br /> File that describes the start locations for agents.  Each line in the file contains a single integer. The first line indicates the number of agents n. The following n lines correspond to the start locations of the n agents.* |
| taskFile               | String <br /> File that describes the locations for tasks.  Each line in the file contains a single integer. The first line indicates the number of tasks m. The following m lines correspond to the locations of the m tasks.\*               |
| numTasksReveal         | Int (=1) <br /> Number of tasks that is                                                                                                                                                                                                        |
| taskAssignmentStrategy | String <br /> The strategy for assigning tasks (“greedy” or “roundrobin”).                                                                                                                                                                     |
| teamSize               | Int <br /> The number of agents consider                                                                                                                                                                                                       |


\* We linearize the a 2-D coordinate and use a single integer to represent a location. 

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
| .                      | free space |
| E                      | emitter point (for ‘delivery’ goal) - traversable              |
| S                      | service point (for ‘pick up’ goal) - traversable                                                                                                                                                                                                        |

## Output JSON Format

The output file of `./lifelong` is a JSON file consisting of the planner output, actual paths of agents, and the statistics.

The following table defines the properties that appear in the output file.

| properties      |                                                                                                                                                                                                                                                                                                                                                                                 |
|-----------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| actionModel     | String <br /> Name for the problem model. Currently, we only have “MAPF_T”.                                                                                                                                                                                                                                                                                                     |
| AllValid        | String <br /> All the actions returned by your planner are valid or not                                                                                                                                                                                                                                                                                                                                                     |
| teamSize        | Int <br /> Number of agents                                                                                                                                                                                                                                                                                                                                                     |
| start           | List <br />A list of start locations. The length of the list is the number of agents.                                                                                                                                                                                                                                                                                           |
| numTaskFinished | Int <br />Number of finished tasks.                                                                                                                                                                                                                                                                                                                                             |
| sumOfCost       | Int <br />                                                                                                                                                                                                                                                                                                                                                                      |
| makespan        | Int <br />                                                                                                                                                                                                                                                                                                                                                                 |
| actualPaths     | List <br /> A list of n strings, where n is the number of agents. Each string represents a sequence of action symbols, separated by commas (“,”).<br />  Action symbols: <br />+ “F”  Forward <br />+ “R” Clockwise rotate <br /> + “C” Counter-clockwise rotate <br />+ “W” Wait <br /> + “T” Implicit wait (only in plannerPaths and corresponding to planner timeout or missing actions) |
| plannerPaths    | List<br /> A list of n strings, where n is the number of agents. Each string represents a sequence of action symbols, separated by commas.                                                                                                                                                                                                                                      |
| plannerTimes    | List <br />A list of planner times in seconds during each planning episode.                                                                                                                                                                                                                                                                          |
| errors          | List <br />A list of errors. Each error is represented by a list [agent1, agent2, timestep, description] where agent1, agent2, and timestep are integers and description is a string. agent1 and agent2 correspond to the id of agents that are involved in the error (agent2=-1 in case there is only one agent involved). The description is a message for the error.         |
| events          | List <br />A list of (task) events. Each event is represented by a list [timestep, id, description] where timestep and id are integers, and description is a string. id corresponds to a task id in tasks. The description can be “assigned” or “finished”.                                                                                                                     |
| tasks           | List <br />A list of tasks. Each task is represented by a list of three integers [id, source, target].                                                                                                                                                                                                                                                                          |


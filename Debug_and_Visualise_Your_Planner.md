# Debug and Visualise Your Planner
The output file provides some related information to help you design and debug your planner. 
Begin by familiarizing yourself with the structure of the output JSON file. Please refer to the [Input_Output_Format.md](./Input_Output_Format.md).

## Debug your planner using Output File
To debug your planners, there are some properties that can help you to learn how your planner coordinate the robots. Here are some suggestions that you might be interested to look at: 
1. Check for Validity: Inspect the `errors` and `AllValid` properties to identify and address any invalid actions.
2. Review Paths: Study the `plannerPaths` and `actualPaths` results to check whether your planner behaves consistently with your expectations or not.
3. Analyze Planner Times: Look at the `plannerTimes` property to understand the time taken by your planner during each planning episode. Investigate further if you notice significant deviations from expected planning times.
4. Other ways: you can also compare your performance and design a better planner by simply looking at the `numTaskFinished` results and analysing the tasks finished by `events` and `tasks`.

## Visualise your planner using Output File
We also provide visualise tools to visualise your plan with the output JSON file. You can visualise those information from the output file:
1. Actual Paths
2. Planner Path
3. Errors
4. Events
For how to use the visualiser, please refer to [Visualiser Page](https://github.com/MAPF-Competition/PlanViz).

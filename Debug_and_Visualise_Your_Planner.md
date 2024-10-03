# Debug and Visualise Your Planner


The output file provides some related information to help you design and debug your planner. 
Begin by familiarizing yourself with the structure of the output JSON file. Please refer to the [Input_Output_Format.md](./Input_Output_Format.md).

## Debug your planner using Output File
To debug your planners, there are some properties that can help you learn how your planner coordinates the robots. Here are some suggestions that you might find useful:
1. Check for Validity: Inspect the `errors` and `scheduleErrors` to identify and address any invalid actions or schedules.
2. Review Paths: Study the `plannerPaths` and `actualPaths` results to check whether your planner behaves consistently with your expectations.
3. Review Schedules: Study the `plannerSchedule` and `actualSchedule` results to check whether the scheduler behaves consistently with your expectations.
4. Analyze Times: Look at the `plannerTimes` property to understand the time taken by your entry during each computing episode. Investigate further if you notice significant deviations from expected time consumption.
5. Other Ways: You can also compare your performance and design a better planner by looking at the `numTaskFinished` results and analyzing the tasks finished by `events` and `tasks`.


**Note that, the visualisation tool PlanVis for start-kit v2.0.0 is currently under development and is coming soon!**

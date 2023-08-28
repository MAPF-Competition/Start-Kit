# Changelog

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

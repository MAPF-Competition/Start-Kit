# Changelog

Version 1.1.0 - 2023-08-04
----------------------------
Added:
- Add changelog.md for tracking version changes
- More example problems for warehouse domain

Changed:
- Updated documentation (adding more descriptions): Input_Output_Format.md, Prepare_Your_Planner.md 
- Add upgrade instruction in README.md
- Add warnings in logger when number of tasks in problem file is smaller than team size

Fixed:
- Fixed issue with example task file for warehouse large only has 1000 tasks (smaller than team size), of which 10000 tasks must be replaced
- Fixed issue with edge conflict checking causing integer overflow bug.

Version 1.0.0 - 2023-07-13
----------------------------
Initial release of the project

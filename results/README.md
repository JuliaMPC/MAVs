# Results Directory
This directory share files between local host and container and it is used to store results data.

The [bag name], set in the demo, should be in format of like demoK_s4_D.bag, where demoK is the demo name, s4 is the case name, and D is the planner name.

To get .csv files with case information:
```
python postProcess.py [bag name] [csv_path_name]
```
where [csv_path_name] is optional, the default value is ./test

# Results Directory
This directory share files between local host and container  
It is used to store results data.
[bag name] should be in format of like demoK_s4_D.bag, where demoK is the demo name, s4 is the case name, and D is the planner name.

to get .csv files with case information:	python run_data.py [bag name] [csv_path_name]
						[csv_path_name] is optional, the default value is ./test

to get .csv files:				python postProcess.py [bag_name1] [bagname2] [bagname3]
						[bag_name1] [bagname2] [bagname3] are optional, the default value is all files in the current folder

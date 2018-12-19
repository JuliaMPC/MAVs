# Results Directory
This directory share files between local host and container  
It is used to store results data
Do not delete this directory

to get .csv files with case information:	python run_data [bag name] [csv_path_name]
						[csv_path_name] is optional, the default value is ./test

to get .csv files:				python postProcess [bag_name1] [bagname2] [bagname3]
						[bag_name1] [bagname2] [bagname3] are optional, the default value is all files in the current folder

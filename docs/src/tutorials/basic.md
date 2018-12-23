# Basic user
There are three things that a basic user should be capable of: running demos, modifying configuration files, and saving results.

## Running demos
First start Docker container in the MAVs folder:
```
sh run.sh
```
Then, the most basic usage of MAVs is simply running the demos. For instance, demoA can be run as:
```
$roslaunch system demoA.launch
```
Note: *package demos documentation is provided in the description of the package*


## Modifying configuration files (i.e. YAML files)
Given that a basic user will often want to test software under different conditions, for instance a different obstacle field, MAVs provides support for this. To accomplish this, MAVs uses YAML files that can easily be changed. For organization, the YAML are all located in the ``MAVs/ros/src/system`` folder. These files include things like obstacle information, i.e. number, radius (currently obstacles are cylinders), etc. In ``.yaml`` form, the obstacle data looks like:
```
 obstacle:
   num: 1
   radius: [1.]
   length: [5.]
   x0: [200]
   y0: [50]
   vx: [0]
   vy: [0]
```
So, these files can be changed and thus the demos, can be tested using different parameterizations.

### github note
The YAML files are currently not ignored by git because they are needed to run MAVs. In the future it may be nice to figure out how to ignore changes in these files, but given that MAVs is under development from a developer's perspective we will not be making that many changes to these files and if they are the same then it will be easier to figure out potential issues.

## Saving results
All of the results are stored in the ``MAVs/results`` folder.

The results are stored using rosbag..TODO..

The demo's can be run and results are the produced and stored even after the MAVs container is terminated.

### github note
A ``.gitignore`` file is added to make sure that when these results are produced they are not submitted to github (assuming a pull request is made afterwards).


## Parameters
Parameters are broken into two categories; `Inputs` and `Outputs`. In the demo, the inputs are also generated, but flags can be set to let the node know that the user will be setting these `rosparams` externally.


## Running multiple tests consecutively
Parameter sweeps can be easily performed in MAVs using the test suite script. This script runs automated test scenarios. Key points to note:

* Test suite has its own set of parameters files and launch scripts
* ``test_main.sh`` is the script you want to run
* In ``test_main.sh`` one can provide multiple nested loops(each to change one single parameter for individual run)
* Results will be stored in ``results/report_test.csv``
* IMPORTANT: If a parameter is updated dynamically inside the ``test_main.sh``, then remove the corresponding entry from the required yaml file. Otherwise value will be overwritten and dynamic parameter update won't work.
* You can modify ``system/config/system/result_model.json`` to specify which parameter or topics you want to save at shutdown
* Delete any old copy of ``report_test.csv`` in results folder, if ``result_model.json`` is modified Otherwise headers won't correspond with new row entries

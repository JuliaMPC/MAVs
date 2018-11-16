# data_logging

## System logger

This demonstrates how we can use rosbag record using a launch file. Details for existing rosbag APIs can be found here [rosbag](http://wiki.ros.org/rosbag/Commandline). For implementation details in launch file, please refer to [Record with rosbag from launch file](https://answers.ros.org/question/52773/record-with-rosbag-from-launch-file/). We record `/nlopcontrol_planner/opt` topic and convert to `.csv` file using `rosbag2csv.py` script.

## Steps

### 1. Add similar snippet to your launch file
Put following code in your demo launch file:

```xml
<?xml version="1.0"?>
<launch>
  <arg name="system_params_path" default="$(find system)/config/system/demos/demoB.yaml"/>

  <!-- Add your nodes -->

  <node pkg="rosbag" type="record" name="record" args="record -O /home/mavs/MAVs/results/opt.bag /nlopcontrol_planner/opt"/>

  <node name="bootstrap" pkg="system" type="bootstrap.jl" output="screen"/>

</launch>
```
Here we specified the topics to record as `args` to `record` node, in this case `opt` topic will be saved in a file called `opt.bag` in `/home/mavs/MAVs/results/`

### 2. Run follwing in cmd prompt
```
$cd /home/mavs/MAVs/results
$python rosbag2csv.py opt.bag
```
This script will create `opt.csv` which is a csv file containing Optimization message  

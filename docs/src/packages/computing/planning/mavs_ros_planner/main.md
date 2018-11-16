# mavs_ros_palnner

This is a setup for [Navigation Stack](http://wiki.ros.org/navigation/Tutorials/Using%20rviz%20with%20the%20Navigation%20Stack#Overview) for the HMMWV model. This uses [move_base](http://wiki.ros.org/move_base) `ROS` package and given a `Goal pose`, it publishes a trajectory into `/cmd_vel`.

The setup uses `navfn/NavfnROS` as `base_global_planner` and `base_local_planner/TrajectoryPlannerROS` as `base_local_planner`. Namespace for this planner is `ros_base_planner`

## Inputs
Following are the argument required by package's main.launch file of `mavs_ros_palnner` package

Name | Description
--- | ---
`system_params_path` | path to cofiguration file that defines system fields. Example file: demoC.yaml
`case_params_path` | path to cofiguration file that defines the case. Example file: case1.yaml
`laser_scan_topic` | topic publishing the lase scan data
`rviz_config_file` | path to a Rviz configuration file

`mavs_ros_planner` heavily relies on the `move_base` framework from ROS. User can also tune parameters related to `cost maps, global/local planners, etc.`` through configuration files present in `MAVs/ros/src/system/config/planner/ros_base_planner`

## Output
Output topics are the same as mentioned in tutorial of [move_base](http://wiki.ros.org/move_base).
Few application specific ros parameters which are publsihed are metnioned below:

Name | Description
--- | ---
`vehicle/chrono/ros_base_planner/traj/x` | list of 'x' part of global coordinates for the created plan
`vehicle/chrono/ros_base_planner/traj/y` | list of 'y' part of global coordinates for the created plan

## Flags and Settings

Name | Description
--- | ---
`/system/ros_base_planner/initialized` | indicates if the planner has successfully published a global plan
`/system/ros_base_planner/goal_known` | indicates wether to use the goal provided in case file or from Rviz

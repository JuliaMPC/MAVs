# demoC

## vehicle_description and ros_base_planner

A demo that shows `ros_base_planner` calculating a path from the LiDAR data collected from the `vehicle_description` package.

### status = working

## To Run
```
roslaunch system demoC.launch
```

## Expected Output
Gazebo would open with the vehicle and Rviz will pop up showing LIDAR scan data.

Case | Description
--- | ---
system/ros_base_planner/flags/goal_known = `false` | User can click on publish goal button in Rviz and select a goal point within the gloabl cost map area. The planner would plan a path from start to goal and display it. It will also update following ros parameters with trajectory data: `vehicle/chrono/ros_base_planner/traj/x`, `vehicle/chrono/ros_base_planner/traj/y`.
system/ros_base_planner/flags/goal_known = `true` | Planner would pick the goal coordinates from the case file. The trajectory would be shown on Rviz and planner will update the ros parameters mentioned above.

Once, the trajectory is published on Rviz, ros parameter `/system/ros_base_planner/initialized` will be set to `true`

### Note
- Goal point has to be within the global cost map for the planner to ba able to plan.
- This is not closed loop, it just shows the a functional ros planner built into the system.

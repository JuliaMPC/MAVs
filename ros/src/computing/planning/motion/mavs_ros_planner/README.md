This is a setup for [Navigation Stack](http://wiki.ros.org/navigation/Tutorials/Using%20rviz%20with%20the%20Navigation%20Stack#Overview) for the HMMWV model. This uses [move_base](http://wiki.ros.org/move_base) ROS package and given a Start Pose and Goal pose, it publishes a trajectory into /cmd_vel.

The setup uses navfn/NavfnROS as base_global_planner and base_local_planner/TrajectoryPlannerROS as base_local_planner

# Steps

- Start Gazebo: roslaunch mavs_gazebo demo.launch
- Run RViz and load the config file from MAVs/ros/src/system/config/planner/ros_base_planner/default.rviz
- Select a 2D Pose Estimate and 2D Nav Goal in RViz. Make sure that Goal is within Global Cost map
- Base planner would create a trajectory which would be visible in RViz

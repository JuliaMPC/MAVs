This package provides a library names libcontact_sensor_mavs.so which can be
used in .sdf files to detect collision for the associated link.

For developers that work on top of Gazebo, do # sudo apt-get install libgazebo8-dev

The plugin declaration accepts an argument called "rosParamName". User can
define the desired ros parameter to update using this.

When a collision is detected, the corrosponding ros parameter is set to true.
By default(when rosparam name is not specified), this plugin defaults to
/vehicle_collided

# Example use case in a .sdf file:

<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>

     <include>
      <uri>model://sun</uri>
    </include>

    <model name="box">
      <link name="link">
        <pose>0 0 0.5 0 0 0</pose>

        <collision name="box_collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>

        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>

        <sensor name="my_contact" type="contact">
          <plugin name="contact_sensor" filename="libcontact_sensor_mavs.so">
            <rosParamName>/vehicle_collided</rosParamName>
          </plugin>
          <contact>
            <collision>box_collision</collision>
          </contact>
          <update_rate>5</update_rate>
        </sensor>
      </link>
    </model>
  </world>
</sdf>


<gazebo reference="base_link" >

</gazebo>


The setup uses navfn/NavfnROS as base_global_planner and base_local_planner/TrajectoryPlannerROS as base_local_planner

 Steps

- Start Gazebo: roslaunch mavs_gazebo demo.launch
- Run RViz and load the config file from MAVs/ros/src/system/config/planner/ros_base_planner/default.rviz
- Select a 2D Pose Estimate and 2D Nav Goal in RViz. Make sure that Goal is within Global Cost map
- Base planner would create a trajectory which would be visible in RViz

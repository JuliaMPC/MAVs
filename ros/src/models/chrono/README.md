# Chrono Vehicle Model
A HMMWV vehicle model developed using Project Chrono is controlled using ROS parameters which transmit a desired path. The vehicle model
is initialized with parameters from a config yaml file, including an initial desired xy path. The vehicle can track to an updated path sent through the
ROS parameter server. This is demonstrated by traj_gen_chrono.cpp updating the ROS parameters for the desired x and y coordinates after the vehicle begins tracking
the initial desired path. The vehicle's states are published in a ROS msg and also saved as ROS parameters.

## Main Packages

 - traj_gen_chrono
 - ros_chrono_traj
 - ros_chrono_traj_msgs

## How to Build

```

$ cd ~/MAVs/ros
$ catkin_make

```
## How to Start

```
$ cd $HOME/MAVs/ros
$ roslaunch ros_chrono_traj path_follower.launch

```
## Change Vehicle Initial Conditions

To change intiial conditions, initial trajectory, or turn off the GUI (work in progress), edit the parameters in the HMMWV YAML config file:

```
$ sudo gedit ros/src/system/chrono/ros_chrono_traj/config/hmmwv_params.yaml

```

## Change Values of Updated Path

Change the values of x2, y2 in traj_gen_chrono.cpp and recompile using catkin_make.

## Monitor Vehicle State

Open another terminal and type:

```
$ rostopic echo vehicleinfo

```
This displays all states and inputs specified in the veh_status.msg file.

## Change ROS Parameter from command line

```
$ rosparam set <param_name> "param_value"

```

## Current Differences between 3DOF Vehicle model and HMMWV model:

### HMMWV Model
- mass: 2,449.55696
- Izz: 3,570.2
- la: 1.871831 (Distance from COM to front axle)
- lb: 1.871831 (Distance from COM to rear axle)

### 3DOF Vehicle Model
- mass: 2,688.7
- Izz: 4,110.1
- la: 1.5775
- lb: 1.7245  

## Parameter list
- /hmmwv_chrono/gui_status (Switch to turn gui on or off)
- /hmmwv_chrono/initial_conditions/ax (Initial x acceleration)
- /hmmwv_chrono/initial_conditions/pitch (Initial pitch)
-/hmmwv_chrono/initial_conditions/r (Initial r)
- /hmmwv_chrono/initial_conditions/roll (Initial roll)
- /hmmwv_chrono/initial_conditions/sa (Initial steering angle)
- /hmmwv_chrono/initial_conditions/ux (Initial x speed)
- /hmmwv_chrono/initial_conditions/v (Initial velocity)
- /hmmwv_chrono/initial_conditions/v_des (Desired velocity)
- /hmmwv_chrono/initial_conditions/x (Initial x)
- /hmmwv_chrono/initial_conditions/y (Initial y)
- /hmmwv_chrono/initial_conditions/yaw (Initial yaw)
- /hmmwv_chrono/initial_conditions/z (Initial z)
- /hmmwv_chrono/traj/x_traj (Desired x path)
- /hmmwv_chrono/traj/y_traj (Desired y path)
- /hmmwv_chrono/vehicle_parameters/Izz (Moment of Inertia about z axis)
- /hmmwv_chrono/vehicle_parameters/la (Distance from COM to front axle)
- /hmmwv_chrono/vehicle_parameters/lb (Distance from COM to rear axle)
- /hmmwv_chrono/vehicle_parameters/mass (Vehicle mass)
- /hmmwv_chrono/vehicleinfo/brk_in (Brake input)
- /hmmwv_chrono/vehicleinfo/sa (Steering angle)
- /hmmwv_chrono/vehicleinfo/str_in (Steering input)
- /hmmwv_chrono/vehicleinfo/t_chrono (Time in chrono model)
- /hmmwv_chrono/vehicleinfo/thrt_in (Throttle input)
- /hmmwv_chrono/vehicleinfo/x_a (X acceleration)
- /hmmwv_chrono/vehicleinfo/x_pos (X position)
- /hmmwv_chrono/vehicleinfo/x_v (X speed)
- /hmmwv_chrono/vehicleinfo/y_pos (Y position)
- /hmmwv_chrono/vehicleinfo/y_v (Y speed)
- /hmmwv_chrono/vehicleinfo/yaw_curr (Yaw)
- /hmmwv_chrono/vehicleinfo/yaw_rate (Yaw rate)

## Topic list
- /vehicleinfo (Vehicle states, inputs, and time)

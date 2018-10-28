# Chrono Vehicle Model
A HMMWV vehicle model developed using Project Chrono is controlled using ROS parameters which transmit a desired path. The vehicle model
is initialized with parameters from a config yaml file, including an initial desired xy path. The vehicle can track to an updated path sent through the
ROS parameter server. This is demonstrated by traj_gen_chrono.cpp updating the ROS parameters for the desired x and y coordinates after the vehicle begins tracking
the initial desired path. The vehicle's states are published in a ROS msg and also saved as ROS parameters.

## Main Packages

 - traj_gen_chrono
 - ros_chrono
 - ros_chrono_msgs

## How to Build
```
$ cd ~/MAVs/ros
$ catkin_make
```
## How to Start
```
$ cd $HOME/MAVs/ros
$ roslaunch ros_chrono path_follower.launch
$ rosparam set system/default/flags/initialized true
```

## Turn off Chrono GUI
To turn off the GUI, change the value of system/chrono/flags/gui to false in test_chrono.yaml.
```
$ sudo gedit ros/src/system/system/test_chrono.yaml
```
## Change Vehicle Initial Conditions

Change initial conditions to x=200.0 in case1.yaml. To change initial trajectory edit the parameters in the hmmwv.yaml config file.

```
$ sudo gedit ros/src/models/chrono/ros_chrono/config/hmmwv_params.yaml
$ sudo gedit ros/src/system/config/vehicle/hmmwv.yaml
$ sudo gedit ros/src/system/config/case1.yaml
```

## Change Values of Updated Path

For the path_follower demo, update the parameters of x2, y2 in traj_gen_chrono.cpp and recompile using catkin_make. Change the system/planner parameter to chrono in test_chrono.yaml. In general, set system/planner to desired planner and update vehicle/chrono/ <planner_name> /traj/x, vehicle/chrono/ <planner_name> /traj/yVal.

## Monitor Vehicle State

Open another terminal and type:
```
$ rostopic echo vehicleinfo
```
This displays all states and inputs specified in the veh_status.msg file.

## View ROS Parameter value from command line

```
$ rosparam get <param_name>
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

## ROS Parameter list
- /system/chrono/flags/gui (Switch to true or false)
- /case/X0/actual/ax (Initial x acceleration)
- /hmmwv_chrono/X0/theta (Initial pitch)
- /case/X0/actual/r (Initial r)
- /hmmwv_chrono/X0/theta (Initial roll)
- /case/X0/actual/sa (Initial steering angle)
- /case/X0/actual/ux (Initial x speed)
- /case/X0/actual/v (Initial velocity)
- /hmmwv_chrono/X0/v_des (Desired velocity)
- /case/X0/actual/x (Initial x)
- /case/X0/actual/yVal (Initial y)
- /case/X0/actual/psi (Initial yaw)
- /hmmwv_chrono/X0/z (Initial z)
- /vehicle/chrono/common/Izz (Moment of Inertia about z axis)
- /vehicle/chrono/common/la (Distance from COM to front axle)
- /vehicle/chrono/common/lb (Distance from COM to rear axle)
- /vehicle/chrono/common/mass (Vehicle mass)
- /vehicle/chrono/control/brk_in (Brake input)
- /vehicle/chrono/state/sa (Steering angle)
- /vehicle/chrono/control/str (Steering input)
- /vehicle/chrono/state/t (Time in chrono model)
- /vehicle/chrono/control/thr (Throttle input)
- /vehicle/chrono/state/ax (X acceleration)
- /vehicle/chrono/state/x (X position)
- /vehicle/chrono/state/ux (X speed)
- /vehicle/chrono/state/yVal (Y position)
- /vehicle/chrono/state/v (Y speed)
- /vehicle/chrono/state/psi (Yaw)
- /vehicle/chrono/state/r (Yaw rate)

## ROS Topic list
- /vehicleinfo (Vehicle states, inputs, and time)

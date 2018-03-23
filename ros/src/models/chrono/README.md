# Chrono Vehicle Model
A HMMWV vehicle model developed using Project Chrono is controlled using ROS parameters which transmit a desired path. The vehicle model
is initialized with parameters from a config yaml file, including an initial desired xy path. The vehicle can track to an updated path sent through the
ROS parameter server. This is demonstrated by traj_gen_chrono.cpp updating the ROS parameters for the desired x and y coordinates after the vehicle begins tracking
the initial desired path. The vehicle's states are published in a ROS msg and also saved as ROS parameters.

Using a reduced HMMWV vehicle assembly with reduced double wishbone suspensions (i.e., suspensions that replace the upper and lower control arms with distance constraints) and rack and pinion steering.

## Reduced HMMWV Model Parameters
### Wheel
See MAVs/ros/src/models/chrono/ros_chrono/src/data/vehicle/hmmwv/wheel folder. The parameters below can be edited in HMMWV_Wheel_FrontLeft.json, HMMWV_Wheel_FrontRight.json, HMMWV_Wheel_RearLeft.json, and HMMWV_Wheel_RearRight.json. Use sudo gedit to overwrite read-only access.

 - Inertia: [0.113, 0.113, 0.113]
 - Mass: 45.4 kg
 - Radius: 0.268 m
 - Width: 0.22 m

### Tire
Currently using rigid tire model (Pacejka model is a work in progress). See Pacejka parameter files in MAVs/ros/src/models/chrono/ros_chrono/src/data/vehicle/tire/HMMWV_pacejka.tir. See rigid parameter files below in MAVs/ros/src/models/chrono/ros_chrono/src/data/vehicle/tire/HMMWV_RigidTire.json. To change these values, edit the files mentioned above. Use sudo gedit to overwrite read-only access.

 - Tire Radius: 0.4699 m
 - Tire Width: 0.254 m

#### Contact Material:
 - Coefficient of Friction: 0.9
 - Coefficient of Restitution: 0.1
 - Young Modulus: 2e7 Pa
 - Poisson Ratio: 0.3
 - Normal Stiffness: 2e5
 - Normal Damping: 40.0
 - Tangential Stiffness: 2e5
 - Tangential Damping: 20.0

### Chassis
See MAVs/ros/src/models/chrono/ros_chrono/src/data/vehicle/hmmwv/HMMWV_Chassis.json to view and edit the parameters below. Use sudo gedit to overwrite read-only access.
 - Mass: 2086.52 kg
 - Moments of Inertia: [1078.52, 2955.66, 3570.20]
 - Products of Inertia: [0, 0, 0]
 - Driver Location: [0, 0.5, 1.2]
 - Driver Orientation: [1, 0, 0, 0]

### Driveline
This model uses rear-wheel drive. See MAVs/ros/src/models/chrono/ros_chrono/src/data/vehicle/hmmwv/driveline/HMMWV_Driveline2WD.json to view and edit the parameters below. Use sudo gedit to overwrite read-only access.

 - Motor Block Direction: [1, 0, 0]
 - Axle Direction: [0,1,0]
 - Driveshaft Inertia: 0.5
 - Differential Box Inertia: 0.6
 - Conical Gear Ratio: -0.2
 - Differential Ratio: -1.0

### Powertrain
This model uses a simplified powertrain with parameters below. SeeMAVs/ros/src/models/chrono/ros_chrono/src/data/vehicle/hmmwv/powertrain/HMMWV_SimplePowertrain.json to view and edit the parameters below. Use sudo gedit to overwrite read-only access.

 - Forward Gear Ratio: 0.3
 - Reverse Gear Ratio: -0.3
 - Max Engine Torque: 1000.0
 - Max Engine Speed: 2000

### Steering
See MAVs/ros/src/models/chrono/ros_chrono/src/data/vehicle/hmmwv/steering/HMMWV_RackPinion.json to view and edit the parameters below. Use sudo gedit to overwrite read-only access.
 - Steering Link Mass: 9.072 kg
 - Steering Link Moments of Inertia: [1,1,1]
 - Steering Link Radius: 0.03 m
 - Steering Link Length: 0.896 m
 - Pinion Radius: 0.1 m
 - Pinion Maximum Angle: 0.87

### Braking
See the AVs/ros/src/models/chrono/ros_chrono/src/data/vehicle/hmmwv/braking folder. Edit HMMWV_BrakeSimple_Front.json and HMMWV_BrakeSimple_Rear.json with sudo gedit to overwrite read-only access.
 - Max Braking Torque: 4000

### Suspension
The reduced suspension model replaces the upper and lower control arms with distance constraints. See MAVs/ros/src/models/chrono/ros_chrono/src/data/vehicle/hmmwv/suspension/HMMWV_DoubleWishboneReducedRear.json to view and edit the parameters below. Use sudo gedit to overwrite read-only access.

 - Spindle Mass: 15.91 kg
 - Spindle Inertia: [2,4,2]
 - Spindle Radius: 0.15 m
 - Spindle Width: 0.06 m
 - Upright Mass: 27.27 kg
 - Upright Inertia: [5,5,5]
 - Upright Radius: 0.025 m
 - Shock Free Length: 0.382 m
 - Spring Coefficient: 369,149.0
 - Damping Coefficient: 63,921.0
 - Axle Inertia: 0.4

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
Change initial condition case/actual/X0/x to x=200.0 in case.yaml. Change system/planner value in global.yaml to default.

```
$ cd $HOME/MAVs/ros
$ roslaunch ros_chrono path_follower.launch
$ rosparam set system/chrono/flags/initialized true
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
- /vehicle/chrono/common/frict_coeff (Coefficient of friction)
- /vehicle/chrono/common/rest_coeff (Coefficient of resitution)
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

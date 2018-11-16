# nlopcontrol_planner

Obstacle avoidance algorithm described [in this paper](https://scholar.google.com/citations?user=aJBQ8dwAAAAJ&hl=en) which can now be solved in real time using an official julia package called [NLOptControl.jl](https://github.com/JuliaMPC/NLOptControl.jl).

If a vehicle needs to drive quickly and avoid moving obstacles then `NLOptControl.jl` is well suited to solve the trajectory planning problem. Where `A*` and `OpenPlanner` are path planning algorithms and are mostly concerned with satisfying kinematic/geometric constraints, they can identify a path to follow, but without a temporal component, they do not tell you how to follow the path. While, trajectory planning also considers how you can follow the path. So, for instance, what are the optimal steering and throttle trajectories (not simply what are the `X` and `Y` waypoints).

`NLOptControl.jl` is designed as a high level tool, so researchers can easily define their own optimization problems, see my documentation [here](https://juliampc.github.io/MPCDocs.jl/latest/index.html). To do this, `nloptcontrol_planner` uses the `MAVs.jl` package. [MAVs.jl](https://github.com/JuliaMPC/MAVs.jl) is a `julia` package that solves an autonomous vehicle motion planning problem using [NLOptControl.jl](https://github.com/JuliaMPC/NLOptControl.jl). There are several different modules, nothing is documented. Read the source code for more info.

## Inputs

### Obstacles

Currently the obstacles are assumed to be represented by circles and their data is to be published to the vectors in the following `rosparam`

Name | Description
--- | ---
`/obstacle/radius` | radius of obstacle in (m)
`/obstacle/vx` | global velocity in global x direction in (m/s)
`/obstacle/vy`| global velocity in global y direction in (m/s)
`/obstacle/x`| current global x (m) position of vehicle in (m)
`/obstacle/y`| current global y (m) position of vehicle in (m)

### Vehicle State
If an actual vehicle is used or an external model of the vehicle is used, `/nloptcontrol_planner/flags/3DOF_plant` should be set to `false`. And the following `rosparam` states (points) should be set:

Name | Description
--- | ---
`/state/x`| global x position (m)
`/state/y`| global y position (m)
`/state/psi`| global heading angle (rad)
`/state/sa`| steering angle at the tire (rad)
`/state/ux`| velocity in the x direction (vehicle frame) in (m/s)
`/state/ax`| acceleration in the x direction (vehicle frame) in (m/s^s)
`/state/r`| yaw rate about the z direction in (rad/s)

## Outputs

### Trajectories
The purpose of this node is to publish reference state trajectories (vectors) in the `Control.msg` as

Name | Description
--- | ---
`/trajectory/t`| time (s)
`/trajectory/x`| global x position trajectory (m)
`/trajectory/y`| global y position trajectory (m)
`/trajectory/psi`| global heading angle trajectory (rad)
`/trajectory/sa`| steering angle trajectory at the tire (rad)
`/trajectory/ux`| speed in the x direction (vehicle frame) (m/s)

### Optimization and MPC message
The error between the predicted initial state and the actual initial state is provided along with some additional optimization information in the `Optimization.msg` as

Type | Name | Description
-- | --- | ---
`float64` | `texP`            |           predicted execution horizon (s)
`float64` | `texA`             |          actual execution horizon (s)
`float64`| `tSolve`             |        optimization time (s)
`string`| `status`               |       optimization status
`float64[]` |`X0p`                |      predicted initial state vector
`float64[]`| `X0a`                 |     actual initial state vector
`float64[]`| `X0e`                  |    error in prediction of initial state vector


## Flags and Settings
Name | Description
--- | ---
`/nloptcontrol_planner/case_name` | name of configuration file for solver settings
`/nloptcontrol_planner/obstacle_name` | name of configuration file for obstacle field
`/nloptcontrol_planner/flags/3DOF_plant` | indicates if the 3DOF plant model in VehicleModels.jl will be used
`/nloptcontrol_planner/flags/init` | indicates if the planner has finished initialization
`/nloptcontrol_planner/flags/known_environment` | indicates if the obstacle information is assumed to be known

## demo
A stand-alone demo to show that the `NLOptControl.jl` is solving the OCP and connected to `ROS`.

### To Run
```
roslaunch nloptcontrol_planner demo.launch
```

### Expected Output
After a few minutes the terminal should display:
```
******************************************************************************
This program contains Ipopt, a library for large-scale nonlinear optimization.
 Ipopt is released as open source code under the Eclipse Public License (EPL).
         For more information visit http://projects.coin-or.org/Ipopt
******************************************************************************

Running model for the: 0 time
[199.831, 8.44829, -0.304456, 0.225719, 1.62274, 0.0524729, 16.7114, -1.11083]
nloptcontrol_planner has been initialized.
Running model for the: 1 time
[198.976, 16.6123, -0.820356, 0.246638, 1.75329, 0.0330637, 16.1247, -1.22459]
Running model for the: 2 time
[197.441, 24.3947, -0.579523, 0.0349721, 1.8273, -0.00889783, 15.6299, -0.77718]
Running model for the: 3 time
[195.715, 31.9384, -0.0419052, -0.121407, 1.80044, -0.0317296, 15.3501, -0.361812]
Running model for the: 4 time
[194.294, 39.4512, 0.237982, -0.16141, 1.72634, -0.0360165, 15.2689, 0.0185792]
Running model for the: 5 time
[193.395, 47.0487, 0.302376, -0.149562, 1.64747, -0.0328082, 15.3683, 0.361693]
Running model for the: 6 time
[193.034, 54.7811, 0.269118, -0.117383, 1.58031, -0.0253441, 15.6244, 0.646153]
Running model for the: 7 time
[193.132, 62.6823, 0.204767, -0.0873723, 1.52942, -0.0189879, 16.0035, 0.854896]
Running model for the: 8 time
[193.601, 70.7833, 0.149901, -0.0649146, 1.49162, -0.0140566, 16.4672, 0.985584]
Running model for the: 9 time
[194.365, 79.1078, 0.10819, -0.0480866, 1.46356, -0.0102397, 16.9765, 1.03786]
Running model for the: 10 time
goal is in range
[195.357, 87.6683, 0.0590599, -0.020078, 1.44614, -0.00305557, 17.4935, 1.02433]
Running model for the: 11 time
goal is in range
[196.478, 96.4715, -0.0230271, 0.00710531, 1.44323, 0.00253812, 18.002, 1.00652]
Running model for the: 12 time
goal is in range
[197.629, 105.525, -0.100926, 0.0220395, 1.45104, 0.00503927, 18.5022, 0.990981]
Running model for the: 13 time
goal is in range
[198.736, 114.834, -0.150177, 0.027248, 1.46365, 0.00574419, 18.9955, 0.978923]
Running model for the: 14 time
goal is in range
[199.758, 124.4, -0.175388, 0.0281426, 1.47758, 0.00578579, 19.4841, 0.974316]
Goal Attained!

[obstacle_avoidance-2] process has finished cleanly
log file: /home/febbo/.ros/log/f7108cac-2de8-11e8-8052-104a7d04da99/obstacle_avoidance-2*.log
```
This indicates a successful test.

### Notes

  * A large optimization problem needs to be initialized
  * caching the functions upon start-up takes a few minutes

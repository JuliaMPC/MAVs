# trajectory_follower

The trajectory_follower package is to control the vehicle by directly applying the output from the `nloptcontrol_planner` package.

## Inputs

Following are the arguments are required by `trajectory_follower.launch` package

#### Trajectories

Name | Description
--- | ---
`/trajectory/t`| time (s)
`/trajectory/x`| global x position trajectory (m)
`/trajectory/y`| global y position trajectory (m)
`/trajectory/psi`| global heading angle trajectory (rad)
`/trajectory/sa`| steering angle trajectory at the tire (rad)
`/trajectory/ux`| speed in the x direction (vehicle frame) (m/s)

## Output

The output of this package is as follows.

#### Vehicle State

Name | Description
--- | ---
`/control/sa`| steering angle at the tire (rad)
`/control/thr`| throttle input, range: [0, 1]
`/control/brk`| break input, range: [0, 1]

## Logic

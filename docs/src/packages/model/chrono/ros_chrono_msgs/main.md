# ros_chrono_msgs
Creates a `ROS` msg file veh_status.msg to observe vehicle states during runtime of `Chrono` model. Used by the ros_chrono package.

## Message format
Name | Variable | Description
--- | ---
`float64` | t_chrono | simulation time
`float64` | x_pos | vehicle x position
`float64` | y_pos | vehicle y position
`float64` | x_v | vehicle velocity in x (m/s)
`float64` | x_a | vehicle acceleration in x (m/s^2)
`float64` | y_v | vehicle velocity in y
`float64` | yaw_curr | current yaw value (rad)
`float64` | yaw_rate | current yaw rate (rad/s)
`float64` | sa | steering angle (rad)
`float64` | thrt_in | throttle control input
`float64` | brk_in | brake control input
`float64` | str_in | steering control input

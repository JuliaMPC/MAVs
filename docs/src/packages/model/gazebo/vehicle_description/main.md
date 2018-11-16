# vehicle_description
Simulates a LiDAR and obstacles in `Gazebo`.

## Input
The vehicle's position and orientation needs to be updated in Gazebo with:

Name | Description
--- | ---
`/state/x`| global x position (m)
`/state/y`| global y position (m)
`/state/psi`| global heading angle (rad)

## Output
rostopic | rosmsg
--- | ---
`/lidar_points`| PointCloud


## Flags
Name | Description
--- | ---
`/vehicle_description/flag/lidar_initialized` | indicates if lidar has been initialized
`/vehicle_description/flag/obstacles_initialized` | indicates if lidar has been initialized
`/vehicle_description/flag/position_update_external` | if this is set to true the position of the vehicle is set internally

## demo
A stand-alone demo to show that the LiDAR model in the `vehicle_description` package is working and the position of the vehicle can be modified.

### To Run
```
roslaunch vehicle_description demo.launch
```

### Expected Output
Gazebo should pop up and the vehicle starts to drive slowly through an obstacle field.

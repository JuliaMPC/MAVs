# Obstacle Detector Description
This obstacle detector is forked from https://github.com/tysik/obstacle_detector.git
We made some modifications so that the package can detect and track obstacles from 3D PointCloud. Detected obstacles come in a form of circles. The working principles of the method are described in an article provided in the resources folder.

# Motivation
This obstacle detection algoritms can predict the postion (x,y), velocty (x,y), and size (assuming circular obstacles).

# Testing
```
$roslaunch obstacle_detector nodelets_test
```
This launch file subscribes PointCloud2, and set obstacle rospama.

# Input
rostopic | rosmsg
--- | ---
/lidar_points| PointCloud2


# Output
Currently the obstacles are assumed to be represented by circles and their data is to be published to the vectors in the following ``rosparam``

Name | Description
--- | ---
`/obstacle/radius` | radius of obstacle in (m)
`/obstacle/vx` | global velocity in global x direction in (m/s)
`/obstacle/vy`| global velocity in global y direction in (m/s)
`/obstacle/x`| current global x (m) position of vehicle in (m)
`/obstacle/y`| current global y (m) position of vehicle in (m)





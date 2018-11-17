# obstacle_detector
This obstacle detector is forked from [obstacle_detector](https://github.com/tysik/obstacle_detector.git)
We made some modifications so that the package can detect and track obstacles from 3D PointCloud. Detected obstacles come in a form of circles. The working principles of the method are described in an article provided in the resources folder.

This obstacle detection algorithms can predict the position `(x,y)`, velocity `(x,y)`, and size (assuming circular obstacles).

## Input
rostopic | rosmsg
--- | ---
/lidar_points| PointCloud2

## Output
Currently the obstacles are assumed to be represented by circles and their data is to be published to the vectors in the following `rosparam`

Name | Description
--- | ---
`/obstacle/radius` | radius of obstacle in (m)
`/obstacle/vx` | global velocity in global x direction in (m/s)
`/obstacle/vy`| global velocity in global y direction in (m/s)
`/obstacle/x`| current global x (m) position of vehicle in (m)
`/obstacle/y`| current global y (m) position of vehicle in (m)

## Settings
Name | Description
--- | ---
`/obstacle_detector/obstacle_extractor/active` | active/sleep mode
`/obstacle_detector/obstacle_extractor/use_scan` | use laser scan messages
`/obstacle_detector/obstacle_extractor/use_pcl` |  use point cloud messages
`/obstacle_detector/obstacle_extractor/use_split_and_merge` | choose wether to use Iterative End Point Fit (false) or Split And Merge (true) algorithm to detect segments
`/obstacle_detector/obstacle_extractor/circles_from_visible` | detect circular obstacles only from fully visible (not occluded) segments
`/obstacle_detector/obstacle_extractor/discard_converted_segments` | do not publish segments, from which the circles were spawned
`/obstacle_detector/obstacle_extractor/min_group_points` | transform the coordinates of obstacles to a frame described with frame_id parameter
`/obstacle_detector/obstacle_extractor/transform_coordinates` | minimum number of points comprising a group to be further processed
`/obstacle_detector/obstacle_extractor/max_group_distance` | if the distance between two points is greater than this value, start a new group,
`/obstacle_detector/obstacle_extractor/distance_proportion` | enlarge the allowable distance between points proportionally to the range of point (use scan angle increment in radians)
`/obstacle_detector/obstacle_extractor/max_split_distance` | if a point in group lays further from a leading line than this value, split the group
`/obstacle_detector/obstacle_extractor/max_merge_separation` | if distance between obstacles is smaller than this value, consider merging them
`/obstacle_detector/obstacle_extractor/max_merge_spread` | merge two segments if all of their extreme points lay closer to the leading line than this value
`/obstacle_detector/obstacle_extractor/max_circle_radius` | if a circle would have greater radius than this value, skip it
`/obstacle_detector/obstacle_extractor/radius_enlargement` | artificially enlarge the circles radius by this value
`/obstacle_detector/obstacle_extractor/frame_id` | name of the coordinate frame used as origin for produced obstacles (used only if transform_coordinates flag is set to true)
`/obstacle_detector/obstacle_tracker/active` | active/sleep mode
`/obstacle_detector/obstacle_tracker/loop_rate` | the main loop rate in Hz
`/obstacle_detector/obstacle_tracker/tracking_duration` | the duration of obstacle tracking in the case of lack of incomming data
`/obstacle_detector/obstacle_tracker/min_correspondence_cost` | a threshold for correspondence test
`/obstacle_detector/obstacle_tracker/std_correspondence_dev` | (experimental) standard deviation of the position ellipse in the correspondence test
`/obstacle_detector/obstacle_tracker/process_variance` | variance of obstacles position and radius (parameter of Kalman Filter)
`/obstacle_detector/obstacle_tracker/process_rate_variance` | variance of rate of change of obstacles values (parameter of Kalman Filter)
`/obstacle_detector/obstacle_tracker/measurement_variance` | variance of measured obstacles values (parameter of Kalman Filter)
`/obstacle_detector/obstacle_tracker/frame_id` | name of the coordinate frame in which the obstacles are described
`/voxel_grid/filter_field_name` | the name of the point field to be used for filtering
`/voxel_grid/filter_limit_min` | The minimum limit of the filter interval
`/voxel_grid/filter_limit_max` | The maximum limit of the filter interval
`/voxel_grid/filter_limit_negative` | Inverts the meaning of the filter interval.
`/voxel_grid/leaf_size` | The extent of a leaf, respectively the voxel size of the result image or the size of the cells which shall accumulate points.

## Flags
Name | Description
--- | ---
`/system/obstacle_detector/flags/running` | indicates whether the obstacle detector is running
`/system/obstacle_detector/flags/initilized` | indicates whether the obstacle detector is initialized

# demo
N/A

var documenterSearchIndex = {"docs": [

{
    "location": "#",
    "page": "Home",
    "title": "Home",
    "category": "page",
    "text": ""
},

{
    "location": "#MAVs-Documentation-1",
    "page": "Home",
    "title": "MAVs Documentation",
    "category": "section",
    "text": "This software simulates an autonomous vehicle within a ROS environment."
},

{
    "location": "#Features-1",
    "page": "Home",
    "title": "Features",
    "category": "section",
    "text": "nonlinear model predictive control through NLOptControl.jl\nlidar simulation through Gazebo\nvehicle model through Chrono"
},

{
    "location": "#Requirements-1",
    "page": "Home",
    "title": "Requirements",
    "category": "section",
    "text": "Tested on Ubuntu Xenial (16.04)An X server\nDocker\nnvidia-docker"
},

{
    "location": "#Installation-1",
    "page": "Home",
    "title": "Installation",
    "category": "section",
    "text": "Clone the develop branch of the repositorygit clone -b develop https://github.com/JuliaMPC/MAVsBuild imagesh build.sh"
},

{
    "location": "#Citation-1",
    "page": "Home",
    "title": "Citation",
    "category": "section",
    "text": "If you find NLOptControl.jl useful, please cite it:@software{nlopt,\n  author = {{Huckleberry Febbo}},\n  title = {NLOptControl.jl},\n  url = {https://github.com/JuliaMPC/NLOptControl.jl},\n  version = {0.0.1},\n  date = {2017-06-17},\n}If you find VehicleModels.jl useful, please cite this paper:@Conference{Febbo2017,\n  author    = {Huckleberry Febbo, Jiechao Liu, Paramsothy Jayakumar, Jeffrey L. Stein, Tulga Ersal},\n  title     = {Moving Obstacle Avoidance for Large, High-Speed Autonomous Ground Vehicles},\n  year      = {2017},\n  publisher = {IEEE}\n}"
},

{
    "location": "tutorials/basic/#",
    "page": "Basic usage",
    "title": "Basic usage",
    "category": "page",
    "text": ""
},

{
    "location": "tutorials/basic/#Basic-user-1",
    "page": "Basic usage",
    "title": "Basic user",
    "category": "section",
    "text": "There are three things that a basic user should be capable of: running demos, modifying configuration files, and saving results."
},

{
    "location": "tutorials/basic/#Running-demos-1",
    "page": "Basic usage",
    "title": "Running demos",
    "category": "section",
    "text": "First start Docker container in the MAVs folder:sh run.shThen, the most basic usage of MAVs is simply running the demos. For instance, demoA can be run as:$roslaunch system demoA.launchNote: package demos documentation is provided in the description of the package"
},

{
    "location": "tutorials/basic/#Modifying-configuration-files-(i.e.-YAML-files)-1",
    "page": "Basic usage",
    "title": "Modifying configuration files (i.e. YAML files)",
    "category": "section",
    "text": "Given that a basic user will often want to test software under different conditions, for instance a different obstacle field, MAVs provides support for this. To accomplish this, MAVs uses YAML files that can easily be changed. For organization, the YAML are all located in the MAVsrossrcsystem folder. These files include things like obstacle information, i.e. number, radius (currently obstacles are cylinders), etc. In yaml form, the obstacle data looks like: obstacle:\n   num: 1\n   radius: [1.]\n   length: [5.]\n   x0: [200]\n   y0: [50]\n   vx: [0]\n   vy: [0]So, these files can be changed and thus the demos, can be tested using different parameterizations."
},

{
    "location": "tutorials/basic/#github-note-1",
    "page": "Basic usage",
    "title": "github note",
    "category": "section",
    "text": "The YAML files are currently not ignored by git because they are needed to run MAVs. In the future it may be nice to figure out how to ignore changes in these files, but given that MAVs is under development from a developer\'s perspective we will not be making that many changes to these files and if they are the same then it will be easier to figure out potential issues."
},

{
    "location": "tutorials/basic/#Saving-results-1",
    "page": "Basic usage",
    "title": "Saving results",
    "category": "section",
    "text": "All of the results are stored in the MAVsresults folder.The results are stored using rosbag..TODO..The demo\'s can be run and results are the produced and stored even after the MAVs container is terminated."
},

{
    "location": "tutorials/basic/#github-note-2",
    "page": "Basic usage",
    "title": "github note",
    "category": "section",
    "text": "A gitignore file is added to make sure that when these results are produced they are not submitted to github (assuming a pull request is made afterwards)."
},

{
    "location": "tutorials/basic/#Parameters-1",
    "page": "Basic usage",
    "title": "Parameters",
    "category": "section",
    "text": "Parameters are broken into two categories; Inputs and Outputs. In the demo, the inputs are also generated, but flags can be set to let the node know that the user will be setting these rosparams externally."
},

{
    "location": "tutorials/basic/#Running-multiple-tests-consecutively-1",
    "page": "Basic usage",
    "title": "Running multiple tests consecutively",
    "category": "section",
    "text": "Parameter sweeps can be easily performed in MAVs using the test suite script. This script runs automated test scenarios. Key points to note:Test suite has its own set of parameters files and launch scripts\ntest_mainsh is an example of a script that can be created. It can be ran as:cd MAVs/ros/src/system/test/main\n./test_main.shIn test_mainsh one can provide multiple nested loops(each to change one single parameter for individual run)\nResults will be stored in resultsreport_testcsv\nIMPORTANT: If a parameter is updated dynamically inside the test_mainsh, then remove the corresponding entry from the required yaml file. Otherwise value will be overwritten and dynamic parameter update won\'t work.\nYou can modify systemconfigsystemresult_modeljson to specify which parameter or topics you want to save at shutdown\nDelete any old copy of report_testcsv in results folder, if result_modeljson is modified Otherwise headers won\'t correspond with new row entries"
},

{
    "location": "tutorials/developer/#",
    "page": "Developing MAVs",
    "title": "Developing MAVs",
    "category": "page",
    "text": ""
},

{
    "location": "tutorials/developer/#Developer-1",
    "page": "Developing MAVs",
    "title": "Developer",
    "category": "section",
    "text": ""
},

{
    "location": "tutorials/developer/#Modifying-files-1",
    "page": "Developing MAVs",
    "title": "Modifying files",
    "category": "section",
    "text": ""
},

{
    "location": "tutorials/developer/#Modifying-files-that-do-not-need-to-be-compiled-1",
    "page": "Developing MAVs",
    "title": "Modifying files that do not need to be compiled",
    "category": "section",
    "text": "There are several file types that can be modified without having to recompile:  YAML files: configuration files Launch files: these files orchestrate the launching the nodes and scripts needed to run the demos Julia and Python files: these files contain the scripts needed to accomplish certain tasks"
},

{
    "location": "tutorials/developer/#Modifying-files-that-need-to-be-compiled-(i.e.-C-files)-1",
    "page": "Developing MAVs",
    "title": "Modifying files that need to be compiled (i.e. C++ files)",
    "category": "section",
    "text": "You can rebuild the workspace with two methods, where the first method takes shorter time and is recommended."
},

{
    "location": "tutorials/developer/#Method-1.-Rebuild-the-ros-workspace-**inside-the-container**:-1",
    "page": "Developing MAVs",
    "title": "Method 1. Rebuild the ros workspace inside the container:",
    "category": "section",
    "text": "$cd ~/MAVs/ros\n$catkin_makeAfter that, you need to commit the change of image. To do this, open a terminal outside of the container and type:$docker commit <container_name> <image_name>Where container_name can be found with:$ docker container lsYou will see something like this:CONTAINER ID        IMAGE               COMMAND                  CREATED             STATUS              PORTS               NAMES\n7607f52ca7a2        mavs                \"/ros_entrypoint.sh …\"   8 minutes ago       Up 8 minutes                            reverent_shtern\nce4096e14bef        mavs                \"/ros_entrypoint.sh …\"   12 hours ago        Up 12 hours                             vigilant_johnsonThe most recently created one, reverent_shtern, is the one you want to commit. Don\'t forget about tab completion.Then you can look at your images with:$ docker image lsWhich gives something like:REPOSITORY          TAG                 IMAGE ID            CREATED             SIZE\n<none>              <none>              46eac405dcb4        8 minutes ago       9.56GB\nmavs                latest              7c00d6949be1        12 hours ago        11.8GBwhere the first image is the failed build obtained with Method 2...Finally, in this example we have:$docker commit reverent_shtern mavs"
},

{
    "location": "tutorials/developer/#Method-2.-Rebuild-the-image-if-container-is-terminated:-1",
    "page": "Developing MAVs",
    "title": "Method 2. Rebuild the image if container is terminated:",
    "category": "section",
    "text": "$sh build.sh"
},

{
    "location": "tutorials/developer/#Potential-issue-1",
    "page": "Developing MAVs",
    "title": "Potential issue",
    "category": "section",
    "text": "File \"/home/mavs/MAVs/ros/src/CMakeLists.txt\" already existsCMake Error: The current CMakeCache.txt directory /home/mavs/MAVs/ros/build/CMakeCache.txt is different than the directory /home/febbo/Documents/workspace/MAVs/ros/build where CMakeCache.txt was created. This may result in binaries being created in the wrong place. If you are not sure, reedit the CMakeCache.txt\nCMake Error: The source \"/home/mavs/MAVs/ros/src/CMakeLists.txt\" does not match the source \"/home/febbo/Documents/workspace/MAVs/ros/src/CMakeLists.txt\" used to generate cache.  Re-run cmake with a different source directory."
},

{
    "location": "tutorials/developer/#Fix-1",
    "page": "Developing MAVs",
    "title": "Fix",
    "category": "section",
    "text": "Remove the cache file OR just use the first method"
},

{
    "location": "tutorials/developer/#Creating-a-demo-1",
    "page": "Developing MAVs",
    "title": "Creating a demo",
    "category": "section",
    "text": "TODO"
},

{
    "location": "tutorials/developer/#Testing-1",
    "page": "Developing MAVs",
    "title": "Testing",
    "category": "section",
    "text": "TODO"
},

{
    "location": "tutorials/developer/#Documentation-1",
    "page": "Developing MAVs",
    "title": "Documentation",
    "category": "section",
    "text": "TODO"
},

{
    "location": "tutorials/developer/#Making-a-pull-request-1",
    "page": "Developing MAVs",
    "title": "Making a pull request",
    "category": "section",
    "text": "TODO"
},

{
    "location": "packages/model/gazebo/vehicle_description/main/#",
    "page": "vehicle_description",
    "title": "vehicle_description",
    "category": "page",
    "text": ""
},

{
    "location": "packages/model/gazebo/vehicle_description/main/#vehicle_description-1",
    "page": "vehicle_description",
    "title": "vehicle_description",
    "category": "section",
    "text": "Simulates a LiDAR and obstacles in Gazebo."
},

{
    "location": "packages/model/gazebo/vehicle_description/main/#Input-1",
    "page": "vehicle_description",
    "title": "Input",
    "category": "section",
    "text": "The vehicle\'s position and orientation needs to be updated in Gazebo with:Name Description\n/state/x global x position (m)\n/state/y global y position (m)\n/state/psi global heading angle (rad)"
},

{
    "location": "packages/model/gazebo/vehicle_description/main/#Output-1",
    "page": "vehicle_description",
    "title": "Output",
    "category": "section",
    "text": "rostopic rosmsg\n/lidar_points PointCloud"
},

{
    "location": "packages/model/gazebo/vehicle_description/main/#Flags-1",
    "page": "vehicle_description",
    "title": "Flags",
    "category": "section",
    "text": "Name Description\n/vehicle_description/flag/lidar_initialized indicates if lidar has been initialized\n/vehicle_description/flag/obstacles_initialized indicates if lidar has been initialized\n/vehicle_description/flag/position_update_external if this is set to true the position of the vehicle is set internally"
},

{
    "location": "packages/model/gazebo/vehicle_description/main/#demo-1",
    "page": "vehicle_description",
    "title": "demo",
    "category": "section",
    "text": "A stand-alone demo to show that the LiDAR model in the vehicle_description package is working and the position of the vehicle can be modified."
},

{
    "location": "packages/model/gazebo/vehicle_description/main/#To-Run-1",
    "page": "vehicle_description",
    "title": "To Run",
    "category": "section",
    "text": "roslaunch vehicle_description demo.launch"
},

{
    "location": "packages/model/gazebo/vehicle_description/main/#Expected-Output-1",
    "page": "vehicle_description",
    "title": "Expected Output",
    "category": "section",
    "text": "Gazebo should pop up and the vehicle starts to drive slowly through an obstacle field."
},

{
    "location": "packages/model/gazebo/contact_sensor/main/#",
    "page": "contact_sensor",
    "title": "contact_sensor",
    "category": "page",
    "text": ""
},

{
    "location": "packages/model/gazebo/contact_sensor/main/#contact_sensor-1",
    "page": "contact_sensor",
    "title": "contact_sensor",
    "category": "section",
    "text": "This package provides a Gazebo plugin: libcontact_sensor_mavs.so that can be used in .sdf files to detect collision of the associated link.For developers that work on top of Gazebo, and wish to update plugins, do: sudo apt-get install libgazebo8-dev"
},

{
    "location": "packages/model/gazebo/contact_sensor/main/#Input-1",
    "page": "contact_sensor",
    "title": "Input",
    "category": "section",
    "text": "Name Description\nrosParamName Ros param that should be updated with truth values for collision detection"
},

{
    "location": "packages/model/gazebo/contact_sensor/main/#Output-1",
    "page": "contact_sensor",
    "title": "Output",
    "category": "section",
    "text": "When a collision is detected, the ros parameter is set to true, it is false at the start. Once a collsion is detected, the ros parameter will always be true."
},

{
    "location": "packages/model/gazebo/contact_sensor/main/#Note-1",
    "page": "contact_sensor",
    "title": "Note",
    "category": "section",
    "text": "When rosParamName is not specified, the plugin defaults to /vehicle_collided"
},

{
    "location": "packages/model/gazebo/contact_sensor/main/#Example-use-case-in-a-.sdf-file:-1",
    "page": "contact_sensor",
    "title": "Example use case in a .sdf file:",
    "category": "section",
    "text": "<?xml version=\"1.0\"?>\n<sdf version=\"1.6\">\n  <world name=\"default\">\n    <include>\n      <uri>model://ground_plane</uri>\n    </include>\n\n     <include>\n      <uri>model://sun</uri>\n    </include>\n\n    <model name=\"box\">\n      <link name=\"link\">\n        <pose>...</pose>\n\n        <collision name=\"box_collision\">\n          ...\n        </collision>\n\n        <visual name=\"visual\">\n          ...\n        </visual>\n\n        <sensor name=\"my_contact\" type=\"contact\">\n          <plugin name=\"contact_sensor\" filename=\"libcontact_sensor_mavs.so\">\n            <rosParamName>/vehicle_collided</rosParamName>\n          </plugin>\n          <contact>\n            <collision>box_collision</collision>\n          </contact>\n          <update_rate>5</update_rate>\n        </sensor>\n      </link>\n    </model>\n  </world>\n</sdf>"
},

{
    "location": "packages/computing/perception/obstacle_detector/main/#",
    "page": "obstacle_detector",
    "title": "obstacle_detector",
    "category": "page",
    "text": ""
},

{
    "location": "packages/computing/perception/obstacle_detector/main/#obstacle_detector-1",
    "page": "obstacle_detector",
    "title": "obstacle_detector",
    "category": "section",
    "text": "This obstacle detector is forked from obstacle_detector We made some modifications so that the package can detect and track obstacles from 3D PointCloud. Detected obstacles come in a form of circles. The working principles of the method are described in an article provided in the resources folder.This obstacle detection algorithms can predict the position (x,y), velocity (x,y), and size (assuming circular obstacles)."
},

{
    "location": "packages/computing/perception/obstacle_detector/main/#Input-1",
    "page": "obstacle_detector",
    "title": "Input",
    "category": "section",
    "text": "rostopic rosmsg\n/lidar_points PointCloud2"
},

{
    "location": "packages/computing/perception/obstacle_detector/main/#Output-1",
    "page": "obstacle_detector",
    "title": "Output",
    "category": "section",
    "text": "Currently the obstacles are assumed to be represented by circles and their data is to be published to the vectors in the following rosparamName Description\n/obstacle/radius radius of obstacle in (m)\n/obstacle/vx global velocity in global x direction in (m/s)\n/obstacle/vy global velocity in global y direction in (m/s)\n/obstacle/x current global x (m) position of vehicle in (m)\n/obstacle/y current global y (m) position of vehicle in (m)"
},

{
    "location": "packages/computing/perception/obstacle_detector/main/#Settings-1",
    "page": "obstacle_detector",
    "title": "Settings",
    "category": "section",
    "text": "Name Description\n/obstacle_detector/obstacle_extractor/active active/sleep mode\n/obstacle_detector/obstacle_extractor/use_scan use laser scan messages\n/obstacle_detector/obstacle_extractor/use_pcl use point cloud messages\n/obstacle_detector/obstacle_extractor/use_split_and_merge choose wether to use Iterative End Point Fit (false) or Split And Merge (true) algorithm to detect segments\n/obstacle_detector/obstacle_extractor/circles_from_visible detect circular obstacles only from fully visible (not occluded) segments\n/obstacle_detector/obstacle_extractor/discard_converted_segments do not publish segments, from which the circles were spawned\n/obstacle_detector/obstacle_extractor/min_group_points transform the coordinates of obstacles to a frame described with frame_id parameter\n/obstacle_detector/obstacle_extractor/transform_coordinates minimum number of points comprising a group to be further processed\n/obstacle_detector/obstacle_extractor/max_group_distance if the distance between two points is greater than this value, start a new group,\n/obstacle_detector/obstacle_extractor/distance_proportion enlarge the allowable distance between points proportionally to the range of point (use scan angle increment in radians)\n/obstacle_detector/obstacle_extractor/max_split_distance if a point in group lays further from a leading line than this value, split the group\n/obstacle_detector/obstacle_extractor/max_merge_separation if distance between obstacles is smaller than this value, consider merging them\n/obstacle_detector/obstacle_extractor/max_merge_spread merge two segments if all of their extreme points lay closer to the leading line than this value\n/obstacle_detector/obstacle_extractor/max_circle_radius if a circle would have greater radius than this value, skip it\n/obstacle_detector/obstacle_extractor/radius_enlargement artificially enlarge the circles radius by this value\n/obstacle_detector/obstacle_extractor/frame_id name of the coordinate frame used as origin for produced obstacles (used only if transform_coordinates flag is set to true)\n/obstacle_detector/obstacle_tracker/active active/sleep mode\n/obstacle_detector/obstacle_tracker/loop_rate the main loop rate in Hz\n/obstacle_detector/obstacle_tracker/tracking_duration the duration of obstacle tracking in the case of lack of incomming data\n/obstacle_detector/obstacle_tracker/min_correspondence_cost a threshold for correspondence test\n/obstacle_detector/obstacle_tracker/std_correspondence_dev (experimental) standard deviation of the position ellipse in the correspondence test\n/obstacle_detector/obstacle_tracker/process_variance variance of obstacles position and radius (parameter of Kalman Filter)\n/obstacle_detector/obstacle_tracker/process_rate_variance variance of rate of change of obstacles values (parameter of Kalman Filter)\n/obstacle_detector/obstacle_tracker/measurement_variance variance of measured obstacles values (parameter of Kalman Filter)\n/obstacle_detector/obstacle_tracker/frame_id name of the coordinate frame in which the obstacles are described\n/voxel_grid/filter_field_name the name of the point field to be used for filtering\n/voxel_grid/filter_limit_min The minimum limit of the filter interval\n/voxel_grid/filter_limit_max The maximum limit of the filter interval\n/voxel_grid/filter_limit_negative Inverts the meaning of the filter interval.\n/voxel_grid/leaf_size The extent of a leaf, respectively the voxel size of the result image or the size of the cells which shall accumulate points."
},

{
    "location": "packages/computing/perception/obstacle_detector/main/#Flags-1",
    "page": "obstacle_detector",
    "title": "Flags",
    "category": "section",
    "text": "Name Description\n/system/obstacle_detector/flags/running indicates whether the obstacle detector is running\n/system/obstacle_detector/flags/initilized indicates whether the obstacle detector is initialized"
},

{
    "location": "packages/computing/perception/obstacle_detector/main/#demo-1",
    "page": "obstacle_detector",
    "title": "demo",
    "category": "section",
    "text": "N/A"
},

{
    "location": "packages/model/chrono/ros_chrono/main/#",
    "page": "ros_chrono",
    "title": "ros_chrono",
    "category": "page",
    "text": ""
},

{
    "location": "packages/model/chrono/ros_chrono/main/#ros_chrono-1",
    "page": "ros_chrono",
    "title": "ros_chrono",
    "category": "section",
    "text": "A HMMWV vehicle model developed in Project Chrono is controlled using ROS parameters by 4 different modes, which are described below."
},

{
    "location": "packages/model/chrono/ros_chrono/main/#Mode-1:-dynamic-path-following-1",
    "page": "ros_chrono",
    "title": "Mode 1: dynamic path following",
    "category": "section",
    "text": "In this mode, the HMMWV vehicle receives and follows dynamic paths with reference speed. The speed control and steering control are achieved internally using PID controllers. To run the HMMWV vehicle using this mode, use the ROS node path_following.$ rosrun ros_chrono path_following"
},

{
    "location": "packages/model/chrono/ros_chrono/main/#Input-1",
    "page": "ros_chrono",
    "title": "Input",
    "category": "section",
    "text": "The input includes the global coordinates of path points and the reference speed obtained from external planners.Name Description\n/trajectory/ux reference vehicle speed (m/s)\n/trajectory/x global x coordinate vector of trajectory points (m)\n/trajectory/y global y coordinate vector of trajectory points (m)"
},

{
    "location": "packages/model/chrono/ros_chrono/main/#Notes-1",
    "page": "ros_chrono",
    "title": "Notes",
    "category": "section",
    "text": "Currently this node only supports straight paths, but will be updated soon to support interpolation and curved paths.For more explanation of the usage, please check the demo at the end of this document."
},

{
    "location": "packages/model/chrono/ros_chrono/main/#Mode-2:-dynamic-steering-trajectory-following-1",
    "page": "ros_chrono",
    "title": "Mode 2: dynamic steering trajectory following",
    "category": "section",
    "text": "In Mode 2, this node steering_controller can simulate the vehicle’s motion at the control of varying steering input. In this test, both steering and velocity command are hardcoded.To run the HMMWV vehicle using this mode, use the ROS node ‘steering_controller’.rosrun ros_chrono steering_controller"
},

{
    "location": "packages/model/chrono/ros_chrono/main/#Input-2",
    "page": "ros_chrono",
    "title": "Input",
    "category": "section",
    "text": "These steering angles obtained from external planners are used to as an input for the Chrono vehicle to follow.Name Description\n/trajectory/ux longitudinal velocity (m/s)\n/trajectory/sa steering angle (rad/s)"
},

{
    "location": "packages/model/chrono/ros_chrono/main/#Notes-2",
    "page": "ros_chrono",
    "title": "Notes",
    "category": "section",
    "text": ""
},

{
    "location": "packages/model/chrono/ros_chrono/main/#Mode-3:-dynamic-speed-trajectory-following-1",
    "page": "ros_chrono",
    "title": "Mode 3: dynamic speed trajectory following",
    "category": "section",
    "text": "In Mode 3, this node velocity_controller can simulate the vehicle’s motion at the control of varying velocity. In this test velocity command is hardcoded.To run the HMMWV vehicle using this mode, use the ROS node ‘velocity_controller’.rosrun ros_chrono velocity_controller"
},

{
    "location": "packages/model/chrono/ros_chrono/main/#Input-3",
    "page": "ros_chrono",
    "title": "Input",
    "category": "section",
    "text": "These velocity trajectories obtained from external planners are used to as an input for the Chrono vehicle to follow.Name Description\n/trajectory/ux longitudinal velocity (m/s)"
},

{
    "location": "packages/model/chrono/ros_chrono/main/#Notes-3",
    "page": "ros_chrono",
    "title": "Notes",
    "category": "section",
    "text": "Below is the expected output for velocity controller: (Image: link)"
},

{
    "location": "packages/model/chrono/ros_chrono/main/#Mode-4:-dynamic-speed-and-steering-trajectory-following-1",
    "page": "ros_chrono",
    "title": "Mode 4: dynamic speed and steering trajectory following",
    "category": "section",
    "text": "In Mode 4, this node steering_controller can simulate the vehicle’s motion at the control of varying velocity and steering input. In this test, both steering and velocity command are hardcoded.To run the HMMWV vehicle using this mode, use the ROS node ‘steering_controller’.rosrun ros_chrono steering_controller"
},

{
    "location": "packages/model/chrono/ros_chrono/main/#Input-4",
    "page": "ros_chrono",
    "title": "Input",
    "category": "section",
    "text": "These velocity and steering angle obtained from external planners are used to generate a path for the Chrono vehicle to follow.Name Description\n/trajectory/ux longitudinal velocity (m/s)\n/trajectory/sa local steering angle (rad/s)"
},

{
    "location": "packages/model/chrono/ros_chrono/main/#Notes-4",
    "page": "ros_chrono",
    "title": "Notes",
    "category": "section",
    "text": "The node rqtgraph for `steeringcontroller` is shown below: (Image: link)"
},

{
    "location": "packages/model/chrono/ros_chrono/main/#All-Modes-1",
    "page": "ros_chrono",
    "title": "All  Modes",
    "category": "section",
    "text": "The following output, settings, flags, topics, and parameters apply to all modes."
},

{
    "location": "packages/model/chrono/ros_chrono/main/#Output-1",
    "page": "ros_chrono",
    "title": "Output",
    "category": "section",
    "text": "The output includes vehicle information stored State.msg and sent to the /state topic as: Name | Description –- | –- /state/t| simulation time (s) /state/x| global vehicle x position (m) /state/y| global vehicle y position (m) /state/ux| vehicle speed in x direction (vehicle frame) (m/s) /state/ax| vehicle acceleration in x (vehicle frame) (m/s^2) /state/v| vehicle speed in y direction (vehicle frame) (m) /state/psi| global yaw angle (rad) /state/r| yaw rate (rad/s) /state/sa| steering angle (rad)The output includes Chrono control information stored in Control.msg and which is sent to the /control as topic as: Name | Description –- | –- /control/t| simulation time (s) /control/throttle| throttle control input [0, +1] /control/brake| brake control input [0, +1] /control/steering| steering control input (rad)To view states updating while Chrono is running, open a new terminal and enter the container by$ docker exec -it <container_name> /bin/bash<container_name> can be auto-filled by the Tab key. Then runTo show the states:$ rostopic echo /stateThis displays all states in the state.msg file.Or:$ rostopic echo /trajectory/chronoThis displays all controls in the controlChrono.msg file."
},

{
    "location": "packages/model/chrono/ros_chrono/main/#Settings-1",
    "page": "ros_chrono",
    "title": "Settings",
    "category": "section",
    "text": "Name Description\nsystem/chrono/flags/gui Disable/Enable Chrono GUI"
},

{
    "location": "packages/model/chrono/ros_chrono/main/#Flags-1",
    "page": "ros_chrono",
    "title": "Flags",
    "category": "section",
    "text": "Name Description\nsystem/chrono/flags/initialized Chrono ROS node is initialized\nsystem/chrono/flags/running Chrono simulation is running"
},

{
    "location": "packages/model/chrono/ros_chrono/main/#Topics-1",
    "page": "ros_chrono",
    "title": "Topics",
    "category": "section",
    "text": "Name Description\n/state vehicle states and time\n/control "
},

{
    "location": "packages/model/chrono/ros_chrono/main/#Parameters-1",
    "page": "ros_chrono",
    "title": "Parameters",
    "category": "section",
    "text": "The following parameters with SI units and angles in radians can be modified:Name Description\n/case/X0/actual/ax Initial x acceleration\n/state/X0/theta Initial pitch\n/case/X0/actual/r Initial yaw rate\n/state/X0/phi Initial roll\n/case/X0/actual/sa Initial steering angle\n/case/X0/actual/ux Initial x speed\n/case/X0/actual/v Initial velocity\n/state/X0/v_des Desired velocity\n/case/X0/actual/x Initial x\n/case/X0/actual/yVal Initial y\n/case/X0/actual/psi Initial yaw\n/state/X0/z Initial z\nvehicle/common/Izz (Moment of Inertia about z axis)\nvehicle/common/la Distance from COM to front axle\n/vehicle/common/lb Distance from COM to rear axle\n/vehicle/common/m Vehicle mass\n/vehicle/common/wheel_radius Wheel radius\n/vehicle/chrono/vehicle_params/frict_coeff Friction Coefficient (Rigid Tire Model)\n/vehicle/chrono/vehicle_params/rest_coeff Restitution Coefficient (Rigid Tire Model)\n/vehicle/chrono/vehicle_params/centroidLoc Chassis centroid location\n/vehicle/chrono/vehicle_params/centroidOrientation Chassis centroid orientation\n/vehicle/chrono/vehicle_params/chassisMass Chassis mass\n/vehicle/chrono/vehicle_params/chassisInertia Chassis inertia\n/vehicle/chrono/vehicle_params/driverLoc Driver location\n/vehicle/chrono/vehicle_params/driverOrientation Driver orientation\n/vehicle/chrono/vehicle_params/motorBlockDirection Motor block direction\n/vehicle/chrono/vehicle_params/axleDirection Axle direction vector\n/vehicle/chrono/vehicle_params/driveshaftInertia Final driveshaft inertia\n/vehicle/chrono/vehicle_params/differentialBoxInertia Differential box inertia\n/vehicle/chrono/vehicle_params/conicalGearRatio Conical gear ratio for steering\n/vehicle/chrono/vehicle_params/differentialRatio Differential ratio\n/vehicle/chrono/vehicle_params/gearRatios Gear ratios (indexed starting from reverse gear ratio and ending at final forward gear ratio)\n/vehicle/chrono/vehicle_params/steeringLinkMass Steering link mass\n/vehicle/chrono/vehicle_params/steeringLinkInertia Steering link inertia\n/vehicle/chrono/vehicle_params/steeringLinkRadius Steering link radius\n/vehicle/chrono/vehicle_params/steeringLinkLength Steering link length\n/vehicle/chrono/vehicle_params/pinionRadius Pinion radius\n/vehicle/chrono/vehicle_params/pinionMaxAngle Pinion max steering angle\n/vehicle/chrono/vehicle_params/maxBrakeTorque Max brake torque"
},

{
    "location": "packages/model/chrono/ros_chrono/main/#demoA-mode-#1:-dynamic-path-following-1",
    "page": "ros_chrono",
    "title": "demoA | mode #1: dynamic path following",
    "category": "section",
    "text": "This demo controls HMMWV vehicle by mode 1, dynamic path following. Two straight paths, together with reference vehicle speed, are alternatively sent to the HMMWV vehicle.To run this demo:$ roslaunch ros_chrono path_following.launch"
},

{
    "location": "packages/model/chrono/ros_chrono/main/#Expected-Output-1",
    "page": "ros_chrono",
    "title": "Expected Output",
    "category": "section",
    "text": "(Image: link)"
},

{
    "location": "packages/model/chrono/ros_chrono/main/#MODE2:-dynamic-steering-trajectory-tracking-1",
    "page": "ros_chrono",
    "title": "MODE2: dynamic steering trajectory tracking",
    "category": "section",
    "text": "[INSERT DESCRIPTION]To run:"
},

{
    "location": "packages/model/chrono/ros_chrono/main/#Expected-Output-2",
    "page": "ros_chrono",
    "title": "Expected Output",
    "category": "section",
    "text": ""
},

{
    "location": "packages/model/chrono/ros_chrono/main/#Notes-5",
    "page": "ros_chrono",
    "title": "Notes",
    "category": "section",
    "text": ""
},

{
    "location": "packages/model/chrono/ros_chrono/main/#MODE3:-dynamic-speed-trajectory-tracking-1",
    "page": "ros_chrono",
    "title": "MODE3: dynamic speed trajectory tracking",
    "category": "section",
    "text": "[INSERT DESCRIPTION]To run:"
},

{
    "location": "packages/model/chrono/ros_chrono/main/#Expected-Output-3",
    "page": "ros_chrono",
    "title": "Expected Output",
    "category": "section",
    "text": ""
},

{
    "location": "packages/model/chrono/ros_chrono/main/#Notes-6",
    "page": "ros_chrono",
    "title": "Notes",
    "category": "section",
    "text": ""
},

{
    "location": "packages/model/chrono/ros_chrono/main/#OLD-(below-this)-1",
    "page": "ros_chrono",
    "title": "OLD (below this)",
    "category": "section",
    "text": "A vehicle model in Chrono that can be used through ROS.The vehicle model currently runs with rigid tire models, a rear-wheel driveline, double wishbone suspension (reduced so that the control arm positions are distance constraints), and rack and pinion steering."
},

{
    "location": "packages/model/chrono/ros_chrono/main/#To-run-1",
    "page": "ros_chrono",
    "title": "To run",
    "category": "section",
    "text": "$ cd $HOME/MAVs/ros\n$ roslaunch ros_chrono demo.launch\n$ rosparam set system/default/flags/initialized true"
},

{
    "location": "packages/model/chrono/ros_chrono/main/#Expected-Output-4",
    "page": "ros_chrono",
    "title": "Expected Output",
    "category": "section",
    "text": "(Image: link)"
},

{
    "location": "packages/model/chrono/ros_chrono/main/#To-run-velocity-test-1",
    "page": "ros_chrono",
    "title": "To run velocity test",
    "category": "section",
    "text": "Velocity test shows the ability of chorono to exchange information with ros.$ roslaunch ros_chrono demo_velocity_control.launch"
},

{
    "location": "packages/model/chrono/ros_chrono/main/#Expected-Output-5",
    "page": "ros_chrono",
    "title": "Expected Output",
    "category": "section",
    "text": ""
},

{
    "location": "packages/model/chrono/ros_chrono/main/#![link](images/demo_velocity_control.png)-1",
    "page": "ros_chrono",
    "title": "(Image: link)",
    "category": "section",
    "text": ""
},

{
    "location": "packages/model/chrono/ros_chrono/main/#To-run-steering-tracking-test-1",
    "page": "ros_chrono",
    "title": "To run steering tracking test",
    "category": "section",
    "text": "steering tracking test shows the ability of chorono to exchange information with ros.$ cd $HOME/MAVs/ros\n$ roslaunch ros_chrono steering.launch"
},

{
    "location": "packages/model/chrono/ros_chrono/main/#Expected-Output-6",
    "page": "ros_chrono",
    "title": "Expected Output",
    "category": "section",
    "text": "(Image: link) Under a constant speed, the vehicle will be controlled by the steering angle command to follow the steering angle trajectory."
},

{
    "location": "packages/model/chrono/ros_chrono/main/#Miscellaneous-notes-(may-be-outdated!)-1",
    "page": "ros_chrono",
    "title": "Miscellaneous notes (may be outdated!)",
    "category": "section",
    "text": ""
},

{
    "location": "packages/model/chrono/ros_chrono/main/#Change-Vehicle-Initial-Conditions-1",
    "page": "ros_chrono",
    "title": "Change Vehicle Initial Conditions",
    "category": "section",
    "text": "To change initial trajectory edit the parameters in the hmmwv_chrono_params.yaml config file.$ sudo gedit ros/src/system/config/s1.yaml\nTo change target speed, edit:$ sudo gedit ros/src/models/chrono/ros_chrono/config/hmmwv_params.yaml"
},

{
    "location": "packages/model/chrono/ros_chrono/main/#Change-Values-of-Updated-Path-1",
    "page": "ros_chrono",
    "title": "Change Values of Updated Path",
    "category": "section",
    "text": "For the pathfollower demo, update the parameters of /state/chrono/default/traj/yVal, /state/chrono/default/traj/x in hmmwvchronoparams.yaml. Change the system/planner parameter to chrono in chrono.yaml. In general, set system/planner to desired planner and update state/chrono/ <plannername> /traj/x, vehicle/chrono/ <planner_name> /traj/yVal."
},

{
    "location": "packages/model/chrono/ros_chrono/main/#Current-Differences-between-3DOF-Vehicle-model-and-HMMWV-model:-1",
    "page": "ros_chrono",
    "title": "Current Differences between 3DOF Vehicle model and HMMWV model:",
    "category": "section",
    "text": "Name 3DOF Chrono Description\nIzz 4,110.1 3,570.2 Inertia about z axis\nla 1.5775 1.871831 Distance from COM to front axle\nlb 1.7245 1.871831 Distance from COM to rear axle\nTire Model PACEJKA RIGID Tire model used by vehicle"
},

{
    "location": "packages/computing/planning/nloptcontrol_planner/main/#",
    "page": "nloptcontrol_planner",
    "title": "nloptcontrol_planner",
    "category": "page",
    "text": ""
},

{
    "location": "packages/computing/planning/nloptcontrol_planner/main/#nlopcontrol_planner-1",
    "page": "nloptcontrol_planner",
    "title": "nlopcontrol_planner",
    "category": "section",
    "text": "Obstacle avoidance algorithm described in this paper which can now be solved in real time using an official julia package called NLOptControl.jl.If a vehicle needs to drive quickly and avoid moving obstacles then NLOptControl.jl is well suited to solve the trajectory planning problem. Where A* and OpenPlanner are path planning algorithms and are mostly concerned with satisfying kinematic/geometric constraints, they can identify a path to follow, but without a temporal component, they do not tell you how to follow the path. While, trajectory planning also considers how you can follow the path. So, for instance, what are the optimal steering and throttle trajectories (not simply what are the X and Y waypoints).NLOptControl.jl is designed as a high level tool, so researchers can easily define their own optimization problems, see my documentation here. To do this, nloptcontrol_planner uses the MAVs.jl package. MAVs.jl is a julia package that solves an autonomous vehicle motion planning problem using NLOptControl.jl. There are several different modules, nothing is documented. Read the source code for more info."
},

{
    "location": "packages/computing/planning/nloptcontrol_planner/main/#Inputs-1",
    "page": "nloptcontrol_planner",
    "title": "Inputs",
    "category": "section",
    "text": ""
},

{
    "location": "packages/computing/planning/nloptcontrol_planner/main/#Obstacles-1",
    "page": "nloptcontrol_planner",
    "title": "Obstacles",
    "category": "section",
    "text": "Currently the obstacles are assumed to be represented by circles and their data is to be published to the vectors in the following rosparamName Description\n/obstacle/radius radius of obstacle in (m)\n/obstacle/vx global velocity in global x direction in (m/s)\n/obstacle/vy global velocity in global y direction in (m/s)\n/obstacle/x current global x (m) position of vehicle in (m)\n/obstacle/y current global y (m) position of vehicle in (m)"
},

{
    "location": "packages/computing/planning/nloptcontrol_planner/main/#Vehicle-State-1",
    "page": "nloptcontrol_planner",
    "title": "Vehicle State",
    "category": "section",
    "text": "If an actual vehicle is used or an external model of the vehicle is used, /nloptcontrol_planner/flags/3DOF_plant should be set to false. And the following rosparam states (points) should be set:Name Description\n/state/x global x position (m)\n/state/y global y position (m)\n/state/psi global heading angle (rad)\n/state/sa steering angle at the tire (rad)\n/state/ux velocity in the x direction (vehicle frame) in (m/s)\n/state/ax acceleration in the x direction (vehicle frame) in (m/s^s)\n/state/r yaw rate about the z direction in (rad/s)"
},

{
    "location": "packages/computing/planning/nloptcontrol_planner/main/#Outputs-1",
    "page": "nloptcontrol_planner",
    "title": "Outputs",
    "category": "section",
    "text": ""
},

{
    "location": "packages/computing/planning/nloptcontrol_planner/main/#Trajectories-1",
    "page": "nloptcontrol_planner",
    "title": "Trajectories",
    "category": "section",
    "text": "The purpose of this node is to publish reference state trajectories (vectors) in the Control.msg asName Description\n/trajectory/t time (s)\n/trajectory/x global x position trajectory (m)\n/trajectory/y global y position trajectory (m)\n/trajectory/psi global heading angle trajectory (rad)\n/trajectory/sa steering angle trajectory at the tire (rad)\n/trajectory/ux speed in the x direction (vehicle frame) (m/s)"
},

{
    "location": "packages/computing/planning/nloptcontrol_planner/main/#Optimization-and-MPC-message-1",
    "page": "nloptcontrol_planner",
    "title": "Optimization and MPC message",
    "category": "section",
    "text": "The error between the predicted initial state and the actual initial state is provided along with some additional optimization information in the Optimization.msg asType | Name | Description – | –- | –- float64 | texP            |           predicted execution horizon (s) float64 | texA             |          actual execution horizon (s) float64| tSolve             |        optimization time (s) string| status               |       optimization status float64[] |X0p                |      predicted initial state vector float64[]| X0a                 |     actual initial state vector float64[]| X0e                  |    error in prediction of initial state vector"
},

{
    "location": "packages/computing/planning/nloptcontrol_planner/main/#Flags-and-Settings-1",
    "page": "nloptcontrol_planner",
    "title": "Flags and Settings",
    "category": "section",
    "text": "Name Description\n/nloptcontrol_planner/case_name name of configuration file for solver settings\n/nloptcontrol_planner/obstacle_name name of configuration file for obstacle field\n/nloptcontrol_planner/flags/3DOF_plant indicates if the 3DOF plant model in VehicleModels.jl will be used\n/nloptcontrol_planner/flags/init indicates if the planner has finished initialization\n/nloptcontrol_planner/flags/known_environment indicates if the obstacle information is assumed to be known"
},

{
    "location": "packages/computing/planning/nloptcontrol_planner/main/#demo-1",
    "page": "nloptcontrol_planner",
    "title": "demo",
    "category": "section",
    "text": "A stand-alone demo to show that the NLOptControl.jl is solving the OCP and connected to ROS."
},

{
    "location": "packages/computing/planning/nloptcontrol_planner/main/#To-Run-1",
    "page": "nloptcontrol_planner",
    "title": "To Run",
    "category": "section",
    "text": "roslaunch nloptcontrol_planner demo.launch"
},

{
    "location": "packages/computing/planning/nloptcontrol_planner/main/#Expected-Output-1",
    "page": "nloptcontrol_planner",
    "title": "Expected Output",
    "category": "section",
    "text": "After a few minutes the terminal should display:******************************************************************************\nThis program contains Ipopt, a library for large-scale nonlinear optimization.\n Ipopt is released as open source code under the Eclipse Public License (EPL).\n         For more information visit http://projects.coin-or.org/Ipopt\n******************************************************************************\n\nRunning model for the: 0 time\n[199.831, 8.44829, -0.304456, 0.225719, 1.62274, 0.0524729, 16.7114, -1.11083]\nnloptcontrol_planner has been initialized.\nRunning model for the: 1 time\n[198.976, 16.6123, -0.820356, 0.246638, 1.75329, 0.0330637, 16.1247, -1.22459]\nRunning model for the: 2 time\n[197.441, 24.3947, -0.579523, 0.0349721, 1.8273, -0.00889783, 15.6299, -0.77718]\nRunning model for the: 3 time\n[195.715, 31.9384, -0.0419052, -0.121407, 1.80044, -0.0317296, 15.3501, -0.361812]\nRunning model for the: 4 time\n[194.294, 39.4512, 0.237982, -0.16141, 1.72634, -0.0360165, 15.2689, 0.0185792]\nRunning model for the: 5 time\n[193.395, 47.0487, 0.302376, -0.149562, 1.64747, -0.0328082, 15.3683, 0.361693]\nRunning model for the: 6 time\n[193.034, 54.7811, 0.269118, -0.117383, 1.58031, -0.0253441, 15.6244, 0.646153]\nRunning model for the: 7 time\n[193.132, 62.6823, 0.204767, -0.0873723, 1.52942, -0.0189879, 16.0035, 0.854896]\nRunning model for the: 8 time\n[193.601, 70.7833, 0.149901, -0.0649146, 1.49162, -0.0140566, 16.4672, 0.985584]\nRunning model for the: 9 time\n[194.365, 79.1078, 0.10819, -0.0480866, 1.46356, -0.0102397, 16.9765, 1.03786]\nRunning model for the: 10 time\ngoal is in range\n[195.357, 87.6683, 0.0590599, -0.020078, 1.44614, -0.00305557, 17.4935, 1.02433]\nRunning model for the: 11 time\ngoal is in range\n[196.478, 96.4715, -0.0230271, 0.00710531, 1.44323, 0.00253812, 18.002, 1.00652]\nRunning model for the: 12 time\ngoal is in range\n[197.629, 105.525, -0.100926, 0.0220395, 1.45104, 0.00503927, 18.5022, 0.990981]\nRunning model for the: 13 time\ngoal is in range\n[198.736, 114.834, -0.150177, 0.027248, 1.46365, 0.00574419, 18.9955, 0.978923]\nRunning model for the: 14 time\ngoal is in range\n[199.758, 124.4, -0.175388, 0.0281426, 1.47758, 0.00578579, 19.4841, 0.974316]\nGoal Attained!\n\n[obstacle_avoidance-2] process has finished cleanly\nlog file: /home/febbo/.ros/log/f7108cac-2de8-11e8-8052-104a7d04da99/obstacle_avoidance-2*.logThis indicates a successful test."
},

{
    "location": "packages/computing/planning/nloptcontrol_planner/main/#Notes-1",
    "page": "nloptcontrol_planner",
    "title": "Notes",
    "category": "section",
    "text": "A large optimization problem needs to be initialized\ncaching the functions upon start-up takes a few minutes"
},

{
    "location": "packages/computing/planning/mavs_ros_planner/main/#",
    "page": "mavsrosplanner",
    "title": "mavsrosplanner",
    "category": "page",
    "text": ""
},

{
    "location": "packages/computing/planning/mavs_ros_planner/main/#mavs*ros*palnner-1",
    "page": "mavsrosplanner",
    "title": "mavsrospalnner",
    "category": "section",
    "text": "This is a setup for Navigation Stack for the HMMWV model. This uses move_base ROS package and given a Goal pose, it publishes a trajectory into /cmd_vel.The setup uses navfn/NavfnROS as base_global_planner and base_local_planner/TrajectoryPlannerROS as base_local_planner. Namespace for this planner is ros_base_planner"
},

{
    "location": "packages/computing/planning/mavs_ros_planner/main/#Inputs-1",
    "page": "mavsrosplanner",
    "title": "Inputs",
    "category": "section",
    "text": "Following are the argument required by package\'s main.launch file of mavs_ros_palnner packageName Description\nsystem_params_path path to cofiguration file that defines system fields. Example file: demoC.yaml\ncase_params_path path to cofiguration file that defines the case. Example file: case1.yaml\nlaser_scan_topic topic publishing the lase scan data\nrviz_config_file path to a Rviz configuration filemavs_ros_planner heavily relies on the move_base framework from ROS. User can also tune parameters related to cost maps, global/local planners, etc.through configuration files present inMAVs/ros/src/system/config/planner/rosbaseplanner`"
},

{
    "location": "packages/computing/planning/mavs_ros_planner/main/#Output-1",
    "page": "mavsrosplanner",
    "title": "Output",
    "category": "section",
    "text": "Output topics are the same as mentioned in tutorial of move_base. Few application specific ros parameters which are publsihed are metnioned below:Name Description\nvehicle/chrono/ros_base_planner/traj/x list of \'x\' part of global coordinates for the created plan\nvehicle/chrono/ros_base_planner/traj/y list of \'y\' part of global coordinates for the created plan"
},

{
    "location": "packages/computing/planning/mavs_ros_planner/main/#Flags-and-Settings-1",
    "page": "mavsrosplanner",
    "title": "Flags and Settings",
    "category": "section",
    "text": "Name Description\n/system/ros_base_planner/initialized indicates if the planner has successfully published a global plan\n/system/ros_base_planner/goal_known indicates wether to use the goal provided in case file or from Rviz"
},

{
    "location": "packages/computing/controller/path_follower/main/#",
    "page": "path_follower",
    "title": "path_follower",
    "category": "page",
    "text": ""
},

{
    "location": "packages/computing/controller/path_follower/main/#path_follower-1",
    "page": "path_follower",
    "title": "path_follower",
    "category": "section",
    "text": "The path_follower package is to control the vehicle using Pure Pursuit (PP) controller. The logic is described as follows."
},

{
    "location": "packages/computing/controller/path_follower/main/#Inputs-1",
    "page": "path_follower",
    "title": "Inputs",
    "category": "section",
    "text": "Following are the arguments are required by path_follower.launch package"
},

{
    "location": "packages/computing/controller/path_follower/main/#Vehicle-State-1",
    "page": "path_follower",
    "title": "Vehicle State",
    "category": "section",
    "text": "Name Description\n/state/x global x position (m)\n/state/y global y position (m)\n/state/psi global heading angle (rad)\n/state/ux velocity in the x direction (vehicle frame) in (m/s)"
},

{
    "location": "packages/computing/controller/path_follower/main/#Trajectories-1",
    "page": "path_follower",
    "title": "Trajectories",
    "category": "section",
    "text": "Name Description\n/trajectory/x global x position trajectory (m)\n/trajectory/y global y position trajectory (m)"
},

{
    "location": "packages/computing/controller/path_follower/main/#Output-1",
    "page": "path_follower",
    "title": "Output",
    "category": "section",
    "text": "The output of this package is as follows."
},

{
    "location": "packages/computing/controller/path_follower/main/#Vehicle-State-2",
    "page": "path_follower",
    "title": "Vehicle State",
    "category": "section",
    "text": "Name Description\n/control/sa steering angle at the tire (rad)\n/control/thr throttle input, range: [0, 1]\n/control/brk break input, range: [0, 1]"
},

{
    "location": "packages/computing/controller/path_follower/main/#Logic-1",
    "page": "path_follower",
    "title": "Logic",
    "category": "section",
    "text": "The Logic of the Pure Pursuit (PP) controller is shown in the figure below.(Image: link)When a path is given to the vehicle to follow, we will first draw an arc   AB with radius R1, and the center of the arc is the vehicle itself. R1 can is chosen to be proportional to the vehicle speed, R1=Kv, with saturation Rmax and Rmin to avoid the case of no solution. Then, find the intersection (point D) between arc AB and the planned path, and express the position of point D as (xD, yD) using the vehicle coordinate system. Point D is the position where the vehicle wants to arrive next. To command the vehicle to head towards point D, we fit another arc CD which passes through the current vehicle position (0, 0) and (xD, yD), and the direction of the arc at (0, 0) is straight up. The radius of the arc CD is R2, which is also the radius of curvature of the real vehicle path. Then, the relationship between R2 and (xD, yD) can be expressed as(Image: link)Then we have(Image: link)The radius of curvature R_2 is closely related to the steering angle as shown in the figure below. Point E is the instantaneous rotation center of the vehicle. (Image: link)Finally, assume there is no slip between tires and the ground. The relationship between the steering angle and the radius of curvature R_2 is(Image: link)Where L is the wheelbase."
},

{
    "location": "packages/computing/controller/trajectory_follower/main/#",
    "page": "trajectory_follower",
    "title": "trajectory_follower",
    "category": "page",
    "text": ""
},

{
    "location": "packages/computing/controller/trajectory_follower/main/#trajectory_follower-1",
    "page": "trajectory_follower",
    "title": "trajectory_follower",
    "category": "section",
    "text": "The trajectoryfollower package is to control the vehicle by directly applying the output from the `nloptcontrolplanner` package."
},

{
    "location": "packages/computing/controller/trajectory_follower/main/#Inputs-1",
    "page": "trajectory_follower",
    "title": "Inputs",
    "category": "section",
    "text": "Following are the arguments are required by trajectory_follower.launch package"
},

{
    "location": "packages/computing/controller/trajectory_follower/main/#Vehicle-State-1",
    "page": "trajectory_follower",
    "title": "Vehicle State",
    "category": "section",
    "text": "Name Description\n/state/ux velocity in the x direction (vehicle frame) in (m/s)"
},

{
    "location": "packages/computing/controller/trajectory_follower/main/#Trajectories-1",
    "page": "trajectory_follower",
    "title": "Trajectories",
    "category": "section",
    "text": "Name Description\n/trajectory/t time (s)\n/trajectory/x global x position trajectory (m)\n/trajectory/y global y position trajectory (m)\n/trajectory/psi global heading angle trajectory (rad)\n/trajectory/sa steering angle trajectory at the tire (rad)\n/trajectory/ux speed in the x direction (vehicle frame) (m/s)"
},

{
    "location": "packages/computing/controller/trajectory_follower/main/#Output-1",
    "page": "trajectory_follower",
    "title": "Output",
    "category": "section",
    "text": "The output of this package is as follows."
},

{
    "location": "packages/computing/controller/trajectory_follower/main/#Vehicle-State-2",
    "page": "trajectory_follower",
    "title": "Vehicle State",
    "category": "section",
    "text": "Name Description\n/control/sa steering angle at the tire (rad)\n/control/thr throttle input, range: [0, 1]\n/control/brk break input, range: [0, 1]"
},

{
    "location": "packages/computing/controller/trajectory_follower/main/#Logic-1",
    "page": "trajectory_follower",
    "title": "Logic",
    "category": "section",
    "text": "The other kind of follower is trajectory follower,  namely longitudinal speed and steering angle control, shown below.(Image: link)The control of the chrono vehicle model utilizes the longitudinal speed and steering angle command from the planner. For longitudinal speed, it is controlled by a closed-loop PID controller which calculates the throttle and braking input into the chrono model. For steering angle, it is under open-loop control which calculates the steering input to the chrono model, where a low-pass filter is implemented to prevent the steering input from changing drastically. "
},

{
    "location": "packages/system/system_shutdown/#",
    "page": "system_shutdown",
    "title": "system_shutdown",
    "category": "page",
    "text": ""
},

{
    "location": "packages/system/system_shutdown/#system_shutdown-1",
    "page": "system_shutdown",
    "title": "system_shutdown",
    "category": "section",
    "text": ""
},

{
    "location": "packages/system/system_shutdown/#System-shutdown-handler-1",
    "page": "system_shutdown",
    "title": "System shutdown handler",
    "category": "section",
    "text": "This is a node in system package that handles system shutdown based on ros parameters"
},

{
    "location": "packages/system/system_shutdown/#Flags-and-Settings-1",
    "page": "system_shutdown",
    "title": "Flags and Settings",
    "category": "section",
    "text": "Name Description\n/system/shutdown/flags/running Shutdown node has been started. Example file: demoD.yaml\n/system/shutdown/flags/initialized Shutdown node has been initialized. Example file: demoD.yaml"
},

{
    "location": "packages/system/system_shutdown/#Inputs-1",
    "page": "system_shutdown",
    "title": "Inputs",
    "category": "section",
    "text": "Following are the argument required by the nodeName Description\n/system/shutdown/params/shutdown_initiation_flags List of parameters that are constantly monitored for their truth values. If any of these becomes true, system shutdown would be initiated. Example file: demoD.yaml\n/system/shutdown/params/shutdown_completion_flags List of parameters which inidicate pre-processing required for a safe shutdown. Only when all the parameters become true, ros system would be shutdown. Example file: demoD.yaml"
},

{
    "location": "packages/system/system_shutdown/#Output-1",
    "page": "system_shutdown",
    "title": "Output",
    "category": "section",
    "text": "If any of the shutdown_initiation_flags becomes true, then a shutdown routine will be initiated. The node would then wait for all shutdown_completion_flags to become true. Once it is achieved, this node would shutdown. Since system_shutdown is always added as a required node in a launch file, the ros system would kill all remaining nodes which were spawned through this launch file."
},

{
    "location": "packages/system/system_shutdown/#Example-use-case-1",
    "page": "system_shutdown",
    "title": "Example use case",
    "category": "section",
    "text": "Put following code in launch file:<?xml version=\"1.0\"?>\n<launch>\n  <arg name=\"system_params_path\" default=\"$(find system)/config/system/demos/demoD.yaml\"/>\n\n  <!-- Add your nodes -->\n\n  <node name=\"system_shutdown\" pkg=\"system\" type=\"system_shutdown\" output=\"screen\" required=\"true\">\n     <rosparam file=\"$(arg system_params_path)\" command=\"load\"/>\n  </node>\n\n  <node name=\"bootstrap\" pkg=\"system\" type=\"bootstrap.jl\" output=\"screen\"/>\n\n</launch>Put following code in your system.yaml. Note: Do not use this directly, this is just an example and shows only relevant components for shutdon node.system:\n flags:\n  override_shutdown_hook: false\n  data_logging_completed: true\n\n shutdown:\n  flags:\n   running: true\n   initialized: false\n  params:\n    shutdown_initiation_flags: [\"system/flags/goal_attained\", \"/vehicle_collided\"]\n    shutdown_completion_flags: [system/flags/data_logging_completed]"
},

{
    "location": "packages/system/data_logging/#",
    "page": "data_logging",
    "title": "data_logging",
    "category": "page",
    "text": ""
},

{
    "location": "packages/system/data_logging/#data_logging-1",
    "page": "data_logging",
    "title": "data_logging",
    "category": "section",
    "text": ""
},

{
    "location": "packages/system/data_logging/#System-logger-1",
    "page": "data_logging",
    "title": "System logger",
    "category": "section",
    "text": "This demonstrates how we can use rosbag record using a launch file. Details for existing rosbag APIs can be found here rosbag. For implementation details in launch file, please refer to Record with rosbag from launch file. We record /nlopcontrol_planner/opt topic and convert to .csv file using rosbag2csv.py script."
},

{
    "location": "packages/system/data_logging/#Steps-1",
    "page": "data_logging",
    "title": "Steps",
    "category": "section",
    "text": ""
},

{
    "location": "packages/system/data_logging/#.-Add-similar-snippet-to-your-launch-file-1",
    "page": "data_logging",
    "title": "1. Add similar snippet to your launch file",
    "category": "section",
    "text": "Put following code in your demo launch file:<?xml version=\"1.0\"?>\n<launch>\n  <arg name=\"system_params_path\" default=\"$(find system)/config/system/demos/demoB.yaml\"/>\n\n  <!-- Add your nodes -->\n\n  <node pkg=\"rosbag\" type=\"record\" name=\"record\" args=\"record -O /home/mavs/MAVs/results/opt.bag /nlopcontrol_planner/opt\"/>\n\n  <node name=\"bootstrap\" pkg=\"system\" type=\"bootstrap.jl\" output=\"screen\"/>\n\n</launch>Here we specified the topics to record as args to record node, in this case opt topic will be saved in a file called opt.bag in /home/mavs/MAVs/results/"
},

{
    "location": "packages/system/data_logging/#.-Run-follwing-in-cmd-prompt-1",
    "page": "data_logging",
    "title": "2. Run follwing in cmd prompt",
    "category": "section",
    "text": "$cd /home/mavs/MAVs/results\n$python rosbag2csv.py opt.bagThis script will create opt.csv which is a csv file containing Optimization message  "
},

{
    "location": "packages/model/chrono/chrono_position_broadcaster/main/#",
    "page": "chronopositionbroadcaster",
    "title": "chronopositionbroadcaster",
    "category": "page",
    "text": ""
},

{
    "location": "packages/model/chrono/chrono_position_broadcaster/main/#chrono*position*broadcaster-1",
    "page": "chronopositionbroadcaster",
    "title": "chronopositionbroadcaster",
    "category": "section",
    "text": "Reformats Chrono vehicle state as a tf/transform message with a orientation quaternion and position vector and broadcasts this message for use by the mavsrosplanner package."
},

{
    "location": "packages/computing/perception/point_cloud_converter/#",
    "page": "pointcloudconverter",
    "title": "pointcloudconverter",
    "category": "page",
    "text": ""
},

{
    "location": "packages/computing/perception/point_cloud_converter/#point*cloud*converter-1",
    "page": "pointcloudconverter",
    "title": "pointcloudconverter",
    "category": "section",
    "text": "The package is documented here.Used to convert PointCloud to PointCloud2. To go from vehicle_description outputs to obstacle_detector inputs.This is taken care of inside one of the obstacle_detector launch file."
},

{
    "location": "packages/system/demos/demoA/#",
    "page": "demoA",
    "title": "demoA",
    "category": "page",
    "text": ""
},

{
    "location": "packages/system/demos/demoA/#demoA-1",
    "page": "demoA",
    "title": "demoA",
    "category": "section",
    "text": ""
},

{
    "location": "packages/system/demos/demoA/#obstacle*detector-with-vehicle*description-1",
    "page": "demoA",
    "title": "obstacledetector with vehicledescription",
    "category": "section",
    "text": "A demo which shows the obstacledetector with vehicledescription."
},

{
    "location": "packages/system/demos/demoA/#status-working-1",
    "page": "demoA",
    "title": "status = working",
    "category": "section",
    "text": ""
},

{
    "location": "packages/system/demos/demoA/#To-Run-1",
    "page": "demoA",
    "title": "To Run",
    "category": "section",
    "text": "roslaunch system demoA.launch"
},

{
    "location": "packages/system/demos/demoA/#Expected-Output-1",
    "page": "demoA",
    "title": "Expected Output",
    "category": "section",
    "text": "Gazebo should pop up, and the vehicle is located at the origin and heading towards y axis. There should be three cylinder obstacles ahead of the vehicle. Rviz should display the pointcloud in blue and the fitted circular obstacles."
},

{
    "location": "packages/system/demos/demoB/#",
    "page": "demoB",
    "title": "demoB",
    "category": "page",
    "text": ""
},

{
    "location": "packages/system/demos/demoB/#demoB-1",
    "page": "demoB",
    "title": "demoB",
    "category": "section",
    "text": ""
},

{
    "location": "packages/system/demos/demoB/#nloptcontrol*planner-with-vehicle*description-1",
    "page": "demoB",
    "title": "nloptcontrolplanner with vehicledescription",
    "category": "section",
    "text": "A demo that shows nloptcontrol_planner moving the vehicle_description vehicle within Gazebo based off of the solution to the OCP every 0.5 s."
},

{
    "location": "packages/system/demos/demoB/#status-working-1",
    "page": "demoB",
    "title": "status = working",
    "category": "section",
    "text": ""
},

{
    "location": "packages/system/demos/demoB/#To-Run-1",
    "page": "demoB",
    "title": "To Run",
    "category": "section",
    "text": "roslaunch system demoB.launch"
},

{
    "location": "packages/system/demos/demoB/#Expected-Output-1",
    "page": "demoB",
    "title": "Expected Output",
    "category": "section",
    "text": "Gazebo should pop up and if you move the view so that you can see to the right (x,y)=(0,200), you will see the vehicle. All of the nodes are thin initialized and the nloptcontrol_planner node takes the longest, so for a few minutes the terminal screen will displaywaiting on obstacle_avoidance.jl in nloptcontrol_planner ...\nwaiting on obstacle_avoidance.jl in nloptcontrol_planner ...\nwaiting on obstacle_avoidance.jl in nloptcontrol_planner ...\nwaiting on obstacle_avoidance.jl in nloptcontrol_planner ...\n......\nRunning model for the: 1 time\n[bootstrap-12] process has finished cleanly\nlog file: /home/tq/.ros/log/60726692-353d-11e8-8a62-b06ebf2c81c1/bootstrap-12*.log\nRunning model for the: 2 time\nRunning model for the: 3 time\nRunning model for the: 4 time\nRunning model for the: 5 time\nRunning model for the: 6 time\nRunning model for the: 7 time\nRunning model for the: 8 time\nRunning model for the: 9 time\nRunning model for the: 10 time\ngoal is in range\nRunning model for the: 11 time\ngoal is in range\nRunning model for the: 12 time\ngoal is in range\nRunning model for the: 13 time\ngoal is in range\nRunning model for the: 14 time\ngoal is in range\nGoal Attained!\n\n[obstacle_avoidance-2] process has finished cleanly\nlog file: /home/tq/.ros/log/60726692-353d-11e8-8a62-b06ebf2c81c1/obstacle_avoidance-2*.logA green path planned by nloptcontrol_planner will be displayed in rviz. Eventually, the controller will be ready and the vehicle will start to move every time a new solution is generated. This is not closed loop, it just shows the connectivity of these nodes within a system."
},

{
    "location": "packages/system/demos/demoI/#",
    "page": "demoI",
    "title": "demoI",
    "category": "page",
    "text": ""
},

{
    "location": "packages/system/demos/demoI/#demoI-1",
    "page": "demoI",
    "title": "demoI",
    "category": "section",
    "text": "In this demo, the vehicle follows a path autonomously, which is generated by nloptcontrol_planner and helps the vehicle to avoid obstacles. The obstacles are assumed to be known which means its position\'s information is directly known by the nloptcontrol_planner. The motion controller used in this demo is Trajectory Follower which is based on the PID. This demo utilizes the command x, y, ux from nloptcontrol_planner."
},

{
    "location": "packages/system/demos/demoI/#To-Run-1",
    "page": "demoI",
    "title": "To Run",
    "category": "section",
    "text": "roslaunch system demoI.launch"
},

{
    "location": "packages/system/demos/demoI/#Expected-Output-1",
    "page": "demoI",
    "title": "Expected Output",
    "category": "section",
    "text": "First figure shows the ui for demoI, where Running model for the: xxx time shows the current step time.  (Image: link)For the numeric results, if you run demoI with case 3 and planner A, B and C, you should expect the reuslts shown in the following figure. In this figure, the black circle shows the 2D geometry of obstacle. The right five figures show velocity, steering angle, velocity control effort, steering control effort and solving time. The solve time depends on the hardware so the real one may different from the one we shown below.(Image: link)Second figure shows the topic maps for demo I where we can see nlopcontrol_planner/control communicate with /obstacle_avoidance and \\path (Image: link)"
},

{
    "location": "packages/system/demos/demoM/#",
    "page": "demoM",
    "title": "demoM",
    "category": "page",
    "text": ""
},

{
    "location": "packages/system/demos/demoM/#demoM-1",
    "page": "demoM",
    "title": "demoM",
    "category": "section",
    "text": "In this demo, the vehicle follows a path autonomously, which is generated by nloptcontrol_planner and helps the vehicle to avoid obstacles. The obstacles\' positions are assumed to be known. The controller in demoM is trajectoryfollower which is based on PID. This demo utilizes the command x, y, ux from `nloptcontrolplanner`. "
},

{
    "location": "packages/system/demos/demoM/#To-Run-1",
    "page": "demoM",
    "title": "To Run",
    "category": "section",
    "text": "$ roslaunch system demoM.launchThis may take a long time to initialize."
},

{
    "location": "packages/system/demos/demoM/#Expected-Output-1",
    "page": "demoM",
    "title": "Expected Output",
    "category": "section",
    "text": "The output of this demo should be similar to the figure below. Running model for the: xxx time shows the current step time. (Image: link)For the numeric results, if you run demoM with case 5 and planner new, you should expect the reuslts shown in the following figure. In this figure, the black circle shows the 2D geometry of obstacle. The right five figures show velocity, steering angle, velocity control effort, steering control effort and solving time. The solve time depends on the hardware so the real one may different from the one we shown below.(Image: link)The relationship between different topics/topics is checked by opening a new terminal, and enter the docker by$ docker exec -it <container_name> /bin/bash<container_name> can be auto filled by the Tab key. Then, open the rqt graph by$ rqt_graphThe output of rqt_graph is shown below. nlopcontrol_planner/control communicates with /obstacle_avoidance.(Image: link)"
},

{
    "location": "packages/system/demos/demoK/#",
    "page": "demoK",
    "title": "demoK",
    "category": "page",
    "text": ""
},

{
    "location": "packages/system/demos/demoK/#demoK-1",
    "page": "demoK",
    "title": "demoK",
    "category": "section",
    "text": "In this demo, the vehicle follows a path autonomously, which is generated by nloptcontrol_planner and helps the vehicle to avoid obstacles. The obstacles are assumed to be known which means its position\'s information is directly known by the nloptcontrol_planner. The motion controller used in this demo is Path Follower which is based on the Pure Pursuit. This demo utilizes the command x, y, ux from nloptcontrol_planner."
},

{
    "location": "packages/system/demos/demoK/#To-Run-1",
    "page": "demoK",
    "title": "To Run",
    "category": "section",
    "text": "$ roslaunch system demoK.launchThis may take a long time to initialize."
},

{
    "location": "packages/system/demos/demoK/#Expected-Output-1",
    "page": "demoK",
    "title": "Expected Output",
    "category": "section",
    "text": "The output of this demo should be similar to the figure below. Running model for the: xxx time shows the current step time. (Image: link)For the numeric results, if you run demoK with case 3 and planner A, B and C, you should expect the reuslts shown in the following figure. In this figure, the black circle shows the 2D geometry of obstacle. The right five figures show velocity, steering angle, velocity control effort, steering control effort and solving time. The solve time depends on the hardware so the real one may different from the one we shown below.(Image: link)The relationship between different topics/topics is checked by opening a new terminal, and enter the docker by$ docker exec -it <container_name> /bin/bash<container_name> can be auto filled by the Tab key. Then, open the rqt graph by$ rqt_graphThe output of rqt_graph is shown below. nlopcontrol_planner/control communicates with /obstacle_avoidance.(Image: link)"
},

{
    "location": "packages/system/demos/demoZ/#",
    "page": "demoZ",
    "title": "demoZ",
    "category": "page",
    "text": ""
},

{
    "location": "packages/system/demos/demoZ/#demoZ-1",
    "page": "demoZ",
    "title": "demoZ",
    "category": "section",
    "text": "In this demo, the vehicle follows a path autonomously, which is generated by nloptcontrol_planner and helps the vehicle to avoid obstacles. The obstacles in this demo is assumed to be unknown, where a 2D LiDAR model is used to detect the position of obstacles and feedback this information to nloptcontrol_planner. The controller in demoZ is pathfollower which is based on Pure Pursuit. This demo utilizes the command x, y, ux from `nloptcontrolplanner`."
},

{
    "location": "packages/system/demos/demoZ/#To-Run-1",
    "page": "demoZ",
    "title": "To Run",
    "category": "section",
    "text": "$ roslaunch system demoZ.launchThis may take a long time to initialize."
},

{
    "location": "packages/system/demos/demoZ/#Expected-Output-1",
    "page": "demoZ",
    "title": "Expected Output",
    "category": "section",
    "text": "The output of this demo should be similar to the figure below. Running model for the: xxx time shows the current step time. (Image: link)For the numeric results, if you run demoZ with case 5 and planner new, you should expect the reuslts shown in the following figure. In this figure, the black circle shows the 2D geometry of obstacle. The right five figures show velocity, steering angle, velocity control effort, steering control effort and solving time. The solve time depends on the hardware so the real one may different from the one we shown below.(Image: link)The relationship between different topics/topics is checked by opening a new terminal, and enter the docker by$ docker exec -it <container_name> /bin/bash<container_name> can be auto filled by the Tab key. Then, open the rqt graph by$ rqt_graphThe output of rqt_graph is shown below. nlopcontrol_planner/control communicates with /obstacle_avoidance.(Image: link)"
},

{
    "location": "docker/workflow/#",
    "page": "Workflow",
    "title": "Workflow",
    "category": "page",
    "text": ""
},

{
    "location": "docker/workflow/#MAVs-Dockerfile-workflow:-1",
    "page": "Workflow",
    "title": "MAVs Dockerfile workflow:",
    "category": "section",
    "text": "The workflow is consisted of two part, build a image and run a container:  "
},

{
    "location": "docker/workflow/#Build-a-Image:-1",
    "page": "Workflow",
    "title": "Build a Image:",
    "category": "section",
    "text": "Pull the image from avpg/cain:middle  \nBuild Dockerfile: this Dockerfile mainly copy the source files in ros/ into the container and run “catkin_make” to setup ros workspace.  "
},

{
    "location": "docker/workflow/#Dockerfile-of-avpg/cain:middle:-1",
    "page": "Workflow",
    "title": "Dockerfile of avpg/cain:middle:",
    "category": "section",
    "text": "Pull the image from avpg/cain:base_cudagl:\nUpdate Julia packages\nClone the source code from JuliaMPC/MAVs and build workspace"
},

{
    "location": "docker/workflow/#Inside-the-Dockerfile-of-avpg/cain:base_cudagl:-1",
    "page": "Workflow",
    "title": "Inside the Dockerfile of avpg/cain:base_cudagl:",
    "category": "section",
    "text": "Build the base image from nvidia/cudagl:9.0-devel-ubuntu16.04\nInstall required packages  \nInstall ROS kinetic  \nInstall Julia\nInstall Chrono  "
},

{
    "location": "docker/workflow/#Run-a-Container:-1",
    "page": "Workflow",
    "title": "Run a Container:",
    "category": "section",
    "text": "Setup PATH variable for later use of volumes when execute docker run.\ndocker run command to execute mavs image.(Image: volume_diagram)"
},

{
    "location": "docker/notes/#",
    "page": "Notes",
    "title": "Notes",
    "category": "page",
    "text": ""
},

{
    "location": "docker/notes/#Docker-notes-1",
    "page": "Notes",
    "title": "Docker notes",
    "category": "section",
    "text": ""
},

{
    "location": "docker/notes/#Basic-Concepts-1",
    "page": "Notes",
    "title": "Basic Concepts",
    "category": "section",
    "text": "An image is a lightweight, stand-alone, executable package that includes everything needed to run a piece of software, including the code, a runtime, libraries, environment variables, and config files.   Docker Hub: A registry of Docker images. You can think of the registry as a directory of all available Docker images. Since the image doesn\'t exist locally, the client will first fetch the image from the registry and then run the image.   Container: A runtime instance of an image. This is a image becomes in memory when actually executed.   Dockerfile: A simple text-file that contains a list of commands that the Docker client calls while creating an image. It\'s a simple way to automate the image creation process.   In an image, a layer is modification to the image, represented by an instruction in the Dockerfile. Layers are applied in sequence to the base image to create the final image. When an image is updated or rebuilt, only layers that change need to be updated, and unchanged layers are cached locally.  "
},

{
    "location": "docker/notes/#Dockerfile-Commands-1",
    "page": "Notes",
    "title": "Dockerfile Commands",
    "category": "section",
    "text": "Command Description\nFROM Load from a base image\nCMD Cmd takes the following format CMD [“excutable”, ”params1”, ”params2”]\nRUN Run is used to execute any commands.\nCOPY Copy the folder/ files from the local host source to the destination in the container.\nENV Set the environment variables.\nUSER Set the (UID) username which is to run the container based on the image being built.\nVOLUME Enable access from your container to a directory on the host machine."
},

{
    "location": "docker/notes/#Docker-Commands-1",
    "page": "Notes",
    "title": "Docker Commands",
    "category": "section",
    "text": "Command Description\ndocker run <image> run a docker container based on the image\ndocker ps show all containers that are currently running\ndocker ps -a show a list of all containers that we ran\ndocker history <image> view all the layers that make up the image once the image is built\ndocker exec -it <container> /bin/bash launch the docker from other terminals\ndocker commit <container> <repository[:TAG]> create a new image from a container’s changes\ndocker kill <container> kill one or more running containers\ndocker rm <container> remove existing container\ndocker image ls list image built on the machine\ndocker image rm <container> remove imagenvidia-docker build -t <container> -f <Dockerfile> nvidia-docker run -it –rm 		automatically remove the container when it exits. –volume list	bind mount a volume –env list		set environment variables -u		user or UID (i.e. mavs) –privileged	give extended privileges to this container"
},

{
    "location": "docker/notes/#Docker-examples:-1",
    "page": "Notes",
    "title": "Docker examples:",
    "category": "section",
    "text": ""
},

{
    "location": "docker/notes/#Stopping-a-running-container:-1",
    "page": "Notes",
    "title": "Stopping a running container:",
    "category": "section",
    "text": "First look the containers:docker container lsthen stop the container that is running for instance:docker rm -f mavs_container"
},

{
    "location": "docker/notes/#Save-changes-made-in-docker-container:-1",
    "page": "Notes",
    "title": "Save changes made in docker container:",
    "category": "section",
    "text": "Find the container ID  $docker ps -lCommit the change$docker commit <container_id> [repository[:tag]]`Change the docker name tag in ./run.sh  "
},

{
    "location": "docker/notes/#Upload-base-image-to-Docker-Hub:-1",
    "page": "Notes",
    "title": "Upload base image to Docker Hub:",
    "category": "section",
    "text": "Tag the image$docker tag <container> [DockerUserName/Repository[:tag]]Push the image to Docker Hub repository$docker push [DockerUserName/Repository[:tag]]"
},

{
    "location": "docker/notes/#Open-up-new-terminals-to-access-Container-(example):-1",
    "page": "Notes",
    "title": "Open up new terminals to access Container (example):",
    "category": "section",
    "text": "Look at the current containers available with$docker container lsCONTAINER ID        IMAGE               COMMAND                  CREATED             STATUS              PORTS               NAMES\nce4096e14bef        mavs                \"/ros_entrypoint.sh …\"   34 minutes ago      Up 34 minutes                           vigilant_johnsonOpen a new terminal$docker exec -it vigilant_johnson /bin/bashNote: you can use “tab” key to auto-complete the <container_name>"
},

{
    "location": "docker/notes/#Docker-hub-1",
    "page": "Notes",
    "title": "Docker hub",
    "category": "section",
    "text": ""
},

{
    "location": "docker/notes/#How-to-update-base-image-on-docker-hub-1",
    "page": "Notes",
    "title": "How to update base image on docker hub",
    "category": "section",
    "text": "Run$sh build.shlogin to docker$docker loginensure the name of the image$docker image lsCurrently this should be avpg_base, where the image name is specified in the build.sh scripttag the image$docker tag avpg_base avpg/cain:base_cudaglassuming that the image name is avpg_basepublish the image$docker push avpg/cain:base_cudaglsee the docker tutorial on containers for more detailed instructions"
},

{
    "location": "docker/notes/#Pull-and-run-the-image-from-the-remote-repository-1",
    "page": "Notes",
    "title": "Pull and run the image from the remote repository",
    "category": "section",
    "text": "It would be interesting to try this with the higher-level image, but this is not yet tested.docker run -p 4000:80 avpg/cain:base_cudagl"
},

{
    "location": "docker/notes/#Running-out-of-space?-1",
    "page": "Notes",
    "title": "Running out of space?",
    "category": "section",
    "text": "Check your current images:docker image lsCheck your current containers:docker image lsDo some trimmingdocker image prune\ndocker container prune\ndocker system prune"
},

{
    "location": "docker/notes/#What-Are-Volumes?-1",
    "page": "Notes",
    "title": "What Are Volumes?",
    "category": "section",
    "text": "For simplicity, volumes are the mechanism for persisting data generated by and used by Docker containers. Click here for more information.   In runsh, several volumes are created. The results/ directory is used to store the demonstration data and we can do some post process after we terminate the container. As for ros/src/ directory, it shares the source files in the local machine with container. That is, we can modify the files outside the container and see the instant change inside the container. And it fits the scenario where we need to update script file, such as Python, ROS launch file, without rebuild the image."
},

{
    "location": "testing/main/#",
    "page": "Testing",
    "title": "Testing",
    "category": "page",
    "text": ""
},

{
    "location": "testing/main/#To-test-(Currently-not-integrated-with-.travis)-1",
    "page": "Testing",
    "title": "To test (Currently not integrated with .travis)",
    "category": "section",
    "text": "Following code will run the test on ros node and library to check if everything is correct. For library gtest, it\'s always true since specific test on each library hasn\'t been finished. Then for the ros node test, it will test demoA.launch as example.$ cd MAVs/ros\n$ catkin_make run_tests"
},

{
    "location": "issues/main/#",
    "page": "Troubleshooting",
    "title": "Troubleshooting",
    "category": "page",
    "text": ""
},

{
    "location": "issues/main/#Potential-Issues-1",
    "page": "Troubleshooting",
    "title": "Potential Issues",
    "category": "section",
    "text": ""
},

{
    "location": "issues/main/#julia-binaries-are-located-in-a-different-folder-than-/usr/bin-1",
    "page": "Troubleshooting",
    "title": "julia binaries are located in a different folder than /usr/bin",
    "category": "section",
    "text": "In such a case you may try to hint to the binary as:#!/opt/julia/bin/env juliaThen after making your script to make it executable with:$ chmod a+x main.jlYou kick off a roscore and run:$ rosrun bot_description main.jlWith the resulting error:/opt/ros/kinetic/bin/rosrun: /home/febbo/catkin_ws/src/bot_description/main.jl: /opt/julia/bin: bad interpreter: Permission denied\n/opt/ros/kinetic/bin/rosrun: line 109: /home/febbo/catkin_ws/src/bot_description/main.jl: SuccessThen after making sure that the binary is in the correct location:$ /opt/julia/bin/julia -e \'println(\"Hello world\")\'\nHello world"
},

{
    "location": "issues/main/#A-fix-is-to-make-a-link-to-the-binary-in-the-usr/bin-directory-1",
    "page": "Troubleshooting",
    "title": "A fix is to make a link to the binary in the usr/bin directory",
    "category": "section",
    "text": "(Image: link)"
},

]}

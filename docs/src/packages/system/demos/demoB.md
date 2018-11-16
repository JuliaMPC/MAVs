# demoB

## nloptcontrol_planner with vehicle_description
A demo that shows `nloptcontrol_planner` moving the `vehicle_description` vehicle within Gazebo based off of the solution to the OCP every `0.5` s.

## status = working

## To Run
```
roslaunch system demoB.launch
```

## Expected Output
Gazebo should pop up and if you move the view so that you can see to the right `(x,y)=(0,200)`, you will see the vehicle. All of the nodes are thin initialized and the `nloptcontrol_planner` node takes the longest, so for a few minutes the terminal screen will display
```
waiting on obstacle_avoidance.jl in nloptcontrol_planner ...
waiting on obstacle_avoidance.jl in nloptcontrol_planner ...
waiting on obstacle_avoidance.jl in nloptcontrol_planner ...
waiting on obstacle_avoidance.jl in nloptcontrol_planner ...
......
Running model for the: 1 time
[bootstrap-12] process has finished cleanly
log file: /home/tq/.ros/log/60726692-353d-11e8-8a62-b06ebf2c81c1/bootstrap-12*.log
Running model for the: 2 time
Running model for the: 3 time
Running model for the: 4 time
Running model for the: 5 time
Running model for the: 6 time
Running model for the: 7 time
Running model for the: 8 time
Running model for the: 9 time
Running model for the: 10 time
goal is in range
Running model for the: 11 time
goal is in range
Running model for the: 12 time
goal is in range
Running model for the: 13 time
goal is in range
Running model for the: 14 time
goal is in range
Goal Attained!

[obstacle_avoidance-2] process has finished cleanly
log file: /home/tq/.ros/log/60726692-353d-11e8-8a62-b06ebf2c81c1/obstacle_avoidance-2*.log
```
A green path planned by nloptcontrol_planner will be displayed in rviz.
Eventually, the controller will be ready and the vehicle will start to move every time a new solution is generated. This is not closed loop, it just shows the connectivity of these nodes within a system.

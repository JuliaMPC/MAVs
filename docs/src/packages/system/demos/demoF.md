# demoF

## nloptcontrol_planner with vehicle_description and chrono
A demo that shows `nloptcontrol_planner` moving the `vehicle_description` vehicle within Gazebo based off of the solution to the OCP every `0.5` s. Chrono takes the trajectory and follow the path/ trajectory, and feedback the states to `vehicle_description`. Now the loop is closed.

## status = working

## To Run
```
roslaunch system demoF.launch
```

## Expected Output
Chrono will pop up when `nloptcontrol_planner` is initialized. The hmmwv and path will display.
The command line output of demoF is similar to demoE, since the ros time is slowed down to chrono time, the planner will be ran for more times.  
Display in Gazebo and Chrono are mirrored in y axis since Gazebo is right-handed and Irrlicht (Chrono gui app) is left-handed.

# demoG

## obstacle detector with nloptcontrol_planner and vehicle_description
Same as `system demoF.launch` except, known_environment is set to false, `obstacle_detector` is used to detect the obstacle information and pass it to `nloptcontrol_planner`.

## status = working??
There are still previous obstacle information in the current frame.  

## To Run
```
roslaunch system demoE.launch
```

## Expected Output
Same as demoF output.

Frame `map` is not published until `nloptcontrol_planner` is initialized.

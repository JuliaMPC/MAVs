# demoM

In this demo, the vehicle follows a path autonomously, which is generated by `nloptcontrol_planner` and helps the vehicle to avoid obstacles. The obstacles' positions are assumed to be known. The controller in demoM is trajectory_follower which is based on PID.
This demo utilizes the command `x`, `y`, `ux` from `nloptcontrol_planner`. 
  
## To Run

```
$ roslaunch system demoM.launch
```

This may take a long time to initialize.

## Expected Output
The output of this demo should be similar to the figure below. `Running model for the: xxx time` shows the current step time. 

![link](demoM/demoM.png)

For the numeric results, if you run demoM with case 5 and planner new, you should expect the reuslts shown in the following figure. In this figure, the black circle shows the 2D geometry of obstacle. The right five figures show velocity, steering angle, velocity control effort, steering control effort and solving time. The solve time depends on the hardware so the real one may different from the one we shown below.

![link](demoM/demoM_s5_new.png)

The relationship between different topics/topics is checked by opening a new terminal, and enter the docker by

```
$ docker exec -it <container_name> /bin/bash
```

`<container_name>` can be auto filled by the `Tab` key. Then, open the rqt graph by

```
$ rqt_graph
```

The output of `rqt_graph` is shown below. `nlopcontrol_planner/control` communicates with `/obstacle_avoidance`.

![link](demoM/demoM_rqt.png)


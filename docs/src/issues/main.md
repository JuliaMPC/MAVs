# Potential Issues



## julia binaries are located in a different folder than `/usr/bin`

In such a case you may try to hint to the binary as:

    #!/opt/julia/bin/env julia

Then after making your script to make it executable with:

    $ chmod a+x main.jl

You kick off a `roscore` and run:

    $ rosrun bot_description main.jl

With the resulting error:

    /opt/ros/kinetic/bin/rosrun: /home/febbo/catkin_ws/src/bot_description/main.jl: /opt/julia/bin: bad interpreter: Permission denied
    /opt/ros/kinetic/bin/rosrun: line 109: /home/febbo/catkin_ws/src/bot_description/main.jl: Success

Then after making sure that the binary is in the correct location:

    $ /opt/julia/bin/julia -e 'println("Hello world")'
    Hello world

#### A fix is to make a link to the binary in the `usr/bin` directory

![link](link3.png)

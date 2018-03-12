#!/usr/bin/env julia

using RobotOS
@rosimport geometry_msgs.msg: Twist

rostypegen()
using geometry_msgs.msg
import geometry_msgs.msg: Twist

# TODO
# 1) check to see if model is paused

"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/28/2018, Last Modified: 2/28/2018 \n
--------------------------------------------------------------------------------------\n
"""
function loop(pub)
    robotNamespace = RobotOS.get_param("robotNamespace")
    obs_num = length(RobotOS.get_param(string(robotNamespace,"/obs/radius")))

    loop_rate = Rate(5.0)

    RobotOS.set_param(string(robotNamespace,"/bool/init_move_obstacles"),true)
    println("obstacle plugin in julia has been initialized.")

    while !is_shutdown()
        for i in 1:obs_num
            cmd = Twist()
            cmd.linear.x = RobotOS.get_param(string(robotNamespace,"/obs/vx"))[i]
            cmd.linear.y = RobotOS.get_param(string(robotNamespace,"/obs/vy"))[i]
            publish(pub[i], cmd)
        end
        rossleep(loop_rate)
    end
end

"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/28/2018, Last Modified: 2/28/2018 \n
--------------------------------------------------------------------------------------\n
"""
function main()
    init_node("rosjl_move_obstacles")
    robotNamespace = RobotOS.get_param("robotNamespace")
    obs_num = length(RobotOS.get_param(string(robotNamespace,"/obs/radius")))

    RobotOS.set_param(string(robotNamespace,"/bool/init_move_obstacles"),false)

    pub = Array{Publisher{Twist}}(obs_num)

    for i in 1:obs_num
        pub[i] = Publisher{Twist}(string("Obstacle",i,"/cmd_vel"), queue_size = 10)
    end

    loop(pub)
end

if ! isinteractive()
    main()
end

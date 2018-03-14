#!/usr/bin/env julia

using RobotOS
@rosimport geometry_msgs.msg: Point, Pose, Pose2D, PoseStamped, Vector3
@rosimport std_srvs.srv: Empty, SetBool
@rosimport nav_msgs.srv.GetPlan
@rosimport gazebo_msgs.msg: ModelState
@rosimport gazebo_msgs.srv: SetModelState, GetModelState, GetWorldProperties

rostypegen()
using geometry_msgs.msg
using std_srvs.srv
using nav_msgs.srv.GetPlan
using gazebo_msgs.msg
using gazebo_msgs.srv
using PyCall
@pyimport tf.transformations as tf

# TODO
# 1) check to see if model is paused

"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/28/2018, Last Modified: 2/28/2018 \n
--------------------------------------------------------------------------------------\n
"""
function loop_straight_line(set_state,get_state)
    loop_rate = Rate(5.0)

    modelName = RobotOS.get_param("vehicle/vehicle_description/robotName")

    RobotOS.set_param("system/vehicle_description/flags/lidar_initialized",true)
    println("lidar simulation in Gazebo has been initialized")
    while !is_shutdown()

        # Get the current position of the Gazebo model
        gs = GetModelStateRequest()
        gs.model_name = modelName
        gs_r = get_state(gs)

        if !gs_r.success
            error(string(" calling /gazebo/get_model_state service: ", gs_r.status_message))
        end

        # Define position to move robot
        vehPose = gs_r.pose  # use the current position
        vehPose.position.x = gs_r.pose.position.x
        vehPose.position.y = gs_r.pose.position.y + 0.1

        # Define the robot state
        ms = ModelState()
        ms.model_name = modelName
        ms.pose = vehPose

        # Set the state of the Gazebo model
        ss = SetModelStateRequest()
        ss.model_state = ms
        ss_r = set_state(ss)

        if !ss_r.success
            error(string(" calling /gazebo/set_model_state service: ", ss_r.status_message))
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
function loop(set_state,get_state)
    loop_rate = Rate(5.0)

    modelName = RobotOS.get_param("vehicle/vehicle_description/robotName")

    RobotOS.set_param("system/vehicle_description/flags/lidar_initialized",true)
    println("lidar simulation in Gazebo has been initialized")
    while !is_shutdown()

        # Define the robot state
        ms = ModelState()
        ms.model_name = modelName
        ms.pose.position.x = RobotOS.get_param("state/x")
        ms.pose.position.y = RobotOS.get_param("state/y")
        Q = tf.quaternion_from_euler(0, 0, RobotOS.get_param("state/psi"))
        ms.pose.orientation.x = Q[1]
        ms.pose.orientation.y = Q[2]
        ms.pose.orientation.z = Q[3]
        ms.pose.orientation.w = Q[4]

        # Set the state of the Gazebo model
        ss = SetModelStateRequest()
        ss.model_state = ms
        ss_r = set_state(ss)

        if !ss_r.success
            error(string(" calling /gazebo/set_model_state service: ", ss_r.status_message))
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
    init_node("rosjl_move_hmmwv")

    # Set up service to get Gazebo model state
    const get_state = ServiceProxy("/gazebo/get_model_state", GetModelState)
    println("Waiting for 'gazebo/get_model_state' service...")
    wait_for_service("gazebo/get_model_state")

    # Set up service to set Gazebo model state
    const set_state = ServiceProxy("/gazebo/set_model_state", SetModelState)
    println("Waiting for 'gazebo/set_model_state' service...")
    wait_for_service("gazebo/set_model_state")

    if !RobotOS.get_param("system/vehicle_description/flags/postion_update_external")
        loop_straight_line(set_state,get_state)
    else
        loop(set_state,get_state)
    end
end

if ! isinteractive()
    main()
end

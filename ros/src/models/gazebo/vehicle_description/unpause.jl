#!/usr/bin/env julia

# NOTE THIS IS NOT FINISHED!!
using RobotOS
@rosimport gazebo_msgs.msg: GetPhysicsProperties
@rosimport gazebo_msgs.srv: GetPhysicsProperties, SetPhysicsProperties

rostypegen()
using geometry_msgs.msg
using gazebo_msgs.srv

function main()
    init_node("unpause_gazebo")

    # Set up service to get Gazebo
    const get_physics = ServiceProxy("/gazebo/get_physics_properties", GetPhysicsProperties)

    println("Waiting for 'gazebo/get_model_state' service...")
    wait_for_service("gazebo/get_model_state")
    # TODO use getparam to get robot name
    pub = Publisher{Pose}("/robot/pose", queue_size=10)

    gp = GetPhysicsProperties()

    loop(get_state, pub)
end

if ! isinteractive()
    main()
end

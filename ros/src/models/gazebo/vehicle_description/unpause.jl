#!/usr/bin/env julia

using RobotOS
@rosimport std_srvs.srv: Empty, SetBool
rostypegen()
using std_srvs.srv

function main()
    init_node("unpause_gazebo")
    ###############################
    # set up services and messages

    # pause simulation
    #const pause_physics = ServiceProxy("/gazebo/pause_physics",Empty)
    #println("Waiting for '/gazebo/pause_physics' service...")
    #wait_for_service("/gazebo/pause_physics")

    # unpause simulation
    const unpause_physics = ServiceProxy("/gazebo/unpause_physics",Empty)
    println("Waiting for '/gazebo/unpause_physics' service...")
    wait_for_service("/gazebo/unpause_physics")

    # set up services and messages
    ###############################

    while(RobotOS.get_param("system/flags/paused"))
    end
    # unpause physics
    up = EmptyRequest()
    unpause_physics(up)

end

if ! isinteractive()
    main()
end

#!/usr/bin/env julia
using RobotOS
@rosimport geometry_msgs.msg: Point, Pose, Pose2D, PoseStamped, Vector3, Twist
@rosimport std_srvs.srv: Empty, SetBool
@rosimport nav_msgs.srv.GetPlan
@rosimport gazebo_msgs.msg: ModelState, ModelStates
@rosimport gazebo_msgs.srv: SetModelState, GetModelState, GetWorldProperties, GetLinkState, SetLinkState

rostypegen()
using geometry_msgs.msg
using std_srvs.srv
using nav_msgs.srv
using nav_msgs.srv.GetPlan
using gazebo_msgs.srv
using gazebo_msgs.msg

function loop(get_link_state, pub)
    loop_rate = Rate(100.0)
    while ! is_shutdown()
        linkName = "hmmwv::base_footprint"  # TODO make this a parameter

        # Get the current position of the Gazebo model
        gs = GetLinkStateRequest()
        gs.link_name = linkName
        gs_r = get_link_state(gs)
        if !gs_r.success
            error(string(" calling /gazebo/get_link_state service: ", gs_r.status_message))
        end

        npt = gs_r.link_state.pose
        publish(pub, npt)
        rossleep(loop_rate)
    end
end

function main()
    init_node("rosjl_link_position")

    # Set up service to get Gazebo link state
    const get_link_state = ServiceProxy("/gazebo/get_link_state",GetLinkState)
    println("Waiting for 'gazebo/get_link_state' service...")
    wait_for_service("gazebo/get_link_state")

    # TODO use getparam to get robot name, also in position_broadcaster.cpp
    pub = Publisher{Pose}("/hmmwv/base_footprint_link_pose", queue_size=10)

    loop(get_link_state, pub)
end

if ! isinteractive()
    main()
end

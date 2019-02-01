#!/usr/bin/env julia
using RobotOS
@rosimport nloptcontrol_planner.msg: Trajectory, Optimization
@rosimport mavs_msgs.msg: control, state
rostypegen()
using nloptcontrol_planner.msg
using mavs_msgs.msg

# NOTE this is inefficient way to use ROS.
# eliminate this node if running off line

# assuming a known environment leads to
    # 1) reduced safety
        # a) more roll-over *
        # b) more crashes *
    # 2) reduced performance
        # a) time to goal *
        # b) steering control effort
    # 3) increased path tracking error
    # 4) increased solve-times *

"""
S = [1., 2., 3., 4.]
NS = 0.5
ts = (S..., NS)
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 1/31/2019, Last Modified: 1/31/2019 \n
--------------------------------------------------------------------------------------\n
"""
function averageSolveTime(msg::Optimization)
    if !RobotOS.has_param("/terms/vector/solveTime")
        ts = msg.tSolve
        RobotOS.set_param("/terms/vector/solveTime", ts)
    else
        ts = ( RobotOS.get_param("/terms/vector/solveTime")..., msg.tSolve )
        RobotOS.set_param("/terms/vector/solveTime", ts)
    end
    RobotOS.set_param("/terms/average/solveTime", sum(ts)/length(ts) )
    return
end

"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 1/31/2019, Last Modified: 1/31/2019 \n
--------------------------------------------------------------------------------------\n
"""
function controlEffort(msg::control)
    # time
    if !RobotOS.has_param("/terms/vector/time")
        t = msg.t
        RobotOS.set_param("/terms/vector/time", t)
    else
        t = ( RobotOS.get_param("/terms/vector/time")..., msg.t )
        RobotOS.set_param("/terms/vector/steeringEffort", t)
    end

    # steering
    if !RobotOS.has_param("/terms/vector/steeringEffort")
        str = msg.str_in^2
        RobotOS.set_param("/terms/vector/steeringEffort", str)
    else
        str = ( RobotOS.get_param("/terms/vector/steeringEffort")..., msg.str_in^2 )
        RobotOS.set_param("/terms/vector/steeringEffort", str)
    end
    # integrate
    se = 0
    for i in 1:length(t)-1
        se = se + str[i]*(t[i+1] - t[i])
    end
    RobotOS.set_param("/terms/steeringEffort", se)

    # throttle effort
    if !RobotOS.has_param("/terms/vector/throttleEffort")
        thrt = msg.thrt_in^2
        RobotOS.set_param("/terms/vector/throttleEffort", thrt)
    else
        thrt = ( RobotOS.get_param("/terms/vector/throttleEffort")..., msg.thrt_in^2 )
        RobotOS.set_param("/terms/vector/throttleEffort", thrt)
    end
    # integrate
    te = 0
    for i in 1:length(t)-1
        te = te + thrt[i]*(t[i+1] - t[i])
    end
    RobotOS.set_param("/terms/throttleEffort", te)

    # braking effort
    if !RobotOS.has_param("/terms/vector/brakingEffort")
        brk = msg.brk_in^2
        RobotOS.set_param("/terms/vector/brakingEffort", brk)
    else
        brk = ( RobotOS.get_param("/terms/vector/brakingEffort")..., msg.brk_in^2 )
        RobotOS.set_param("/terms/vector/brakingEffort", brk)
    end
    # integrate
    be = 0
    for i in 1:length(t)-1
        be = be + brk[i]*(t[i+1] - t[i])
    end
    RobotOS.set_param("/terms/brakingEffort", be)

    return nothing
end

"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/1/2019, Last Modified: 2/1/2019 \n
--------------------------------------------------------------------------------------\n
"""
function statePts(msg::state)
    if !RobotOS.has_param("/terms/vector/xv")
        xv = (0., msg.x)
        yv = (0., msg.y)
        RobotOS.set_param("/terms/vector/xv", msg.x)
        RobotOS.set_param("/terms/vector/yv", msg.y)
    else
        xv = ( RobotOS.get_param("/terms/vector/xv"), msg.x )
        yv = ( RobotOS.get_param("/terms/vector/yv"), msg.y )
        RobotOS.set_param("/terms/vector/xv", msg.x )
        RobotOS.set_param("/terms/vector/yv", msg.y )
    end
    return nothing
end

"""
# waypoints
px = [1., 2., 3., 4.]
py = [1., 2.5, 3.2, 5.]
# previous [1] and current [2] x state of vehicle
xv = [2., 2.1]
yv = [1.5, 1.7]

px = [0., 1.]
py = [1., 1.]
xv = [0.001, .5]
yv = [0.001, 0.002]

D = (px.- xv[2]).^2 + (py.-yv[2]).^2
idxA = indmin(D)
idxB = indmin(D[1:end .!= idxA])
m1 = (px[idxB] - px[idxA])/(py[idxB] - py[idxA] + eps())
m2 = (yv[2] - yv[1])/(xv[2] - xv[1] + eps())

# orientation error
ae = atan( (m2 - m1)/(1 + m2*m1 + eps()) )

# calculate the intersection between the two lines
x = (yv[1] - m2*xv[1] - py[idxA] + m1*px[idxA])/(m1 - m2 + eps())
y = m1*x + py[1] - m1*px[1]

# need distance R between intersection point of two lines and the vehicle's current state
R = ( (x - xv[2])^2 + (y - yv[2])^2 )^0.5

# tracking error
te = R*sin(ae)

# could do a parameter sweep on the look-ahead distance in the planner and the scaling factor on speed
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 1/31/2019, Last Modified: 1/31/2019 \n
--------------------------------------------------------------------------------------\n
"""
function loop()
    loop_rate = Rate(5.0)

    while(!RobotOS.has_param("/terms/vector/xv") && !RobotOS.has_param("/trajectory/y"))
    end

    RobotOS.set_param("system/flags/terms_initialized",true)
    println("calcualte terms has been initialized")
    while(RobotOS.get_param("system/flags/paused"))
    end

    # calcuates trackingError
    while !is_shutdown()
        if false # TODO finish
            xv = RobotOS.get_param("/terms/vector/xv")
            yv = RobotOS.get_param("/terms/vector/yv")
            px = RobotOS.get_param("/trajectory/x")
            py = RobotOS.get_param("/trajectory/y")

            # calculations
            D = (px.- xv[2]).^2 + (py.-yv[2]).^2
            idxA = indmin(D)
            idxB = indmin(D[1:end .!= idxA])
            m1 = (px[idxB] - px[idxA])/(py[idxB] - py[idxA] + eps())
            m2 = (yv[2] - yv[1])/(xv[2] - xv[1] + eps())

            # orientation error
            oe = atan( (m2 - m1)/(1 + m2*m1 + eps()) )
            if !RobotOS.has_param("/terms/vector/oe")
                oe = oe^2
                RobotOS.set_param("/terms/vector/oe", oe)
            else
                oe = ( RobotOS.get_param("/terms/vector/oe")..., oe^2 )
                RobotOS.set_param("/terms/vector/oe", oe)
            end
            RobotOS.set_param("/terms/oeAve", mean(oe))

            # calculate the intersection between the two lines
            x = (yv[1] - m2*xv[1] - py[idxA] + m1*px[idxA])/(m1 - m2 + eps())
            y = m1*x + py[1] - m1*px[1]

            # need distance R between intersection point of two lines and the vehicle's current state
            R = ( (x - xv[2])^2 + (y - yv[2])^2 )^0.5

            # tracking error
            te = R*sin(oe)
            if !RobotOS.has_param("/terms/vector/te")
                te = te^2
                RobotOS.set_param("/terms/vector/te", te)
            else
                te = ( RobotOS.get_param("/terms/vector/te")..., te^2 )
                RobotOS.set_param("/terms/vector/te", te)
            end
            RobotOS.set_param("/terms/teAve", mean(te))
        end
        rossleep(loop_rate)  # sleep for leftover time
    end
end
"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 1/31/2019, Last Modified: 1/31/2019 \n
--------------------------------------------------------------------------------------\n
"""
function main()
    println("initializing calculateTerms node ...")
    init_node("calculateTerms")
    plannerNamespace = RobotOS.get_param("system/nloptcontrol_planner/namespace")
    subA = Subscriber{Optimization}(string(plannerNamespace, "/opt"), averageSolveTime, queue_size = 10)
#    subB = Subscriber{control}("/control", controlEffort, queue_size = 10)
#    subC = Subscriber{state}("/state", statePts, queue_size = 10)
    loop()
end

if !isinteractive()
    main()
end

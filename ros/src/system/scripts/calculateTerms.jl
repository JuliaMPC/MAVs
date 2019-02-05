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
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 1/31/2019, Last Modified: 1/31/2019 \n
--------------------------------------------------------------------------------------\n
"""
function averageSolveTime(msg::Optimization)
    currentTs = msg.tSolve
    if !RobotOS.has_param("/terms/average/solveTime")
        RobotOS.set_param("/terms/average/solveTime", currentTs)
        RobotOS.set_param("/terms/number/solveTime", 1)
    else
        prevAve = RobotOS.get_param("/terms/average/solveTime")
        prevNum = RobotOS.get_param("/terms/number/solveTime")
        ave = (prevAve*prevNum + msg.tSolve)/(prevNum + 1)
        RobotOS.set_param("/terms/average/solveTime", ave)
        RobotOS.set_param("/terms/number/solveTime", prevNum + 1)
    end

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
    if !RobotOS.has_param("/terms/control/t")
        dt = 0;
        RobotOS.set_param("/terms/control/t", msg.t)
    else
        dt = msg.t - RobotOS.get_param("/terms/control/t")
    end
    RobotOS.set_param("/terms/control/t", msg.t)

    # steering effort
    if !RobotOS.has_param("/terms/steeringEffort")
        sePrev = 0
    else
        sePrev = RobotOS.get_param("/terms/steeringEffort")
        if isnan(sePrev); sePrev = 0; end
    end
    RobotOS.get_param("/terms/control/t")
    RobotOS.set_param("/terms/steeringEffort", sePrev + msg.str_in^2*dt)

    # throttle effort
    if !RobotOS.has_param("/terms/throttleEffort")
        tePrev = 0
    else
        tePrev = RobotOS.get_param("/terms/throttleEffort")
        if isnan(tePrev); tePrev = 0; end
    end
    RobotOS.set_param("/terms/throttleEffort", tePrev + msg.thrt_in^2*dt)

    # braking effort
    if !RobotOS.has_param("/terms/brakingEffort")
        brkPrev = 0
    else
        brkPrev = RobotOS.get_param("/terms/brakingEffort")
        if isnan(brkPrev); brkPrev = 0; end
    end
    RobotOS.set_param("/terms/brakingEffort", brkPrev + msg.brk_in^2*dt)

    return nothing
end

"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/1/2019, Last Modified: 2/1/2019 \n
--------------------------------------------------------------------------------------\n
"""
function statePts(msg::state)
    if !RobotOS.has_param("/terms/xv")
        RobotOS.set_param("/terms/xv",(0., msg.x))
        RobotOS.set_param("/terms/yv", (0., msg.y))
    else
        RobotOS.set_param("/terms/xv", ( RobotOS.get_param("/terms/xv")[end], msg.x ))
        RobotOS.set_param("/terms/yv", ( RobotOS.get_param("/terms/yv")[end], msg.y ))
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

px = [0., 0.]
py = [0., 1.]
xv = [0., 0.]
yv = [0., 0.002]

px = [0., 1.]
py = [0., 1.]
xv = [0., 0.]
yv = [0., 0.002]

px = [0., 1.]
py = [1., 1.]
xv = [0.001, .5]
yv = [0.001, 0.002]

px = [-0.0359484, -0.0350867, -0.0296149, -0.0215991, -0.0116513, 0.000304115, 0.0145289, 0.0313392, 0.0510726, 0.0740774, 0.100708, 0.131319, 0.166263, 0.205882, 0.250503, 0.300433, 0.355952, 0.417305, 0.484701, 0.558301, 0.638217, 0.724506, 0.817161, 0.916109, 1.02121, 1.13223, 1.2489, 1.37083, 1.4976, 1.62876, 1.76389]
py =  [19.6402, 22.069, 24.5229, 26.9977, 29.4915, 32.004, 34.5353, 37.0858, 39.656, 42.2463, 44.857, 47.4881, 50.1397, 52.8117, 55.5036, 58.215, 60.9452, 63.6933, 66.4581, 69.2383, 72.0321, 74.8375, 77.6518, 80.4719, 83.2939, 86.1132, 88.9246, 91.7222, 94.4998, 97.2513, 99.9719]
xv = [-0.0365072, -0.0365088]
yv = [21.2213, 21.2554]

D = (px.- xv[2]).^2 + (py.-yv[2]).^2
idxA = indmin(D)
D[idxA] = NaN
idxB = indmin(D)
# one of the closest points can be ahead and the other may be behind etc.
m1 = (py[idxB] - py[idxA])/(px[idxB] - px[idxA] + eps())
m2 = (yv[2] - yv[1])/(xv[2] - xv[1] + eps())

# orientation error
ae = atan( (m1 - m2)/(1 + m1*m2 + eps()) )
aeDegrees = abs(ae*180/pi)

# calculate the intersection between the two lines
x = (yv[1] - m2*xv[1] - py[idxA] + m1*px[idxA])/(m1 - m2 + eps())
y = m1*x + py[1] - m1*px[1]

# need distance R between intersection point of two lines and the vehicle's current state
R = ( (x - xv[2])^2 + (y - yv[2])^2 )^0.5

# tracking error
te = abs(R*sin(ae))

# could do a parameter sweep on the look-ahead distance in the planner and the scaling factor on speed
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 1/31/2019, Last Modified: 1/31/2019 \n
--------------------------------------------------------------------------------------\n
"""
function loop()
    loop_rate = Rate(5.0)

    while(!RobotOS.has_param("/terms/xv"))
        rossleep(Rate(0.5))
    end
    while(!RobotOS.has_param("/trajectory/x"))
        rossleep(Rate(0.5))
    end

    RobotOS.set_param("system/flags/terms_initialized",true)
    println("calcualte terms has been initialized")

    while(RobotOS.get_param("system/flags/paused"))
    end

    # calcuates trackingError
    while !is_shutdown()  && !RobotOS.get_param("/system/flags/done") && !RobotOS.get_param("/vehicle_collided")
        xv = RobotOS.get_param("/terms/xv")
        yv = RobotOS.get_param("/terms/yv")
        px = RobotOS.get_param("/trajectory/x")
        py = RobotOS.get_param("/trajectory/y")

        # calculations
        # one of the closest points can be ahead and the other may be behind etc.
        D = (px.- xv[2]).^2 + (py.-yv[2]).^2
        idxA = indmin(D)
        D[idxA] = NaN
        idxB = indmin(D)
        m1 = (py[idxB] - py[idxA])/(px[idxB] - px[idxA] + eps())
        m2 = (yv[2] - yv[1])/(xv[2] - xv[1] + eps())

        # orientation error
        currentOe = atan( (m1 - m2)/(1 + m1*m2 + eps()) )
        if !RobotOS.has_param("/terms/average/orientationError")
            RobotOS.set_param("/terms/average/orientationError", abs(currentOe*180/pi))
            RobotOS.set_param("/terms/number/orientationError", 1)
        else
            prevAve = RobotOS.get_param("/terms/average/orientationError")
            prevNum = RobotOS.get_param("/terms/number/orientationError")
            ave = (prevAve*prevNum + abs(currentOe*180/pi))/(prevNum + 1)
            RobotOS.set_param("/terms/average/orientationError", ave)
            RobotOS.set_param("/terms/number/orientationError", prevNum + 1)
        end

        # calculate the intersection between the two lines
        x = (yv[1] - m2*xv[1] - py[idxA] + m1*px[idxA])/(m1 - m2 + eps())
        y = m1*x + py[1] - m1*px[1]

        # need distance R between intersection point of two lines and the vehicle's current state
        R = ( (x - xv[2])^2 + (y - yv[2])^2 )^0.5

        # tracking error
        if !RobotOS.has_param("/terms/average/trackingError")
            RobotOS.set_param("/terms/average/trackingError",  abs(R*sin(currentOe)))
            RobotOS.set_param("/terms/number/trackingError", 1)
        else
            prevAve = RobotOS.get_param("/terms/average/trackingError")
            prevNum = RobotOS.get_param("/terms/number/trackingError")
            ave = (prevAve*prevNum + abs(R*sin(currentOe)))/(prevNum + 1)
            RobotOS.set_param("/terms/average/trackingError", ave)
            RobotOS.set_param("/terms/number/trackingError", prevNum + 1)
        end
    #    @show RobotOS.get_param("/terms/average/orientationError")
    #    @show RobotOS.get_param("/terms/average/trackingError")
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
    subB = Subscriber{control}("/control", controlEffort, queue_size = 10)
    subC = Subscriber{state}("/state", statePts, queue_size = 10)
    loop()
end

if !isinteractive()
    main()
end

#!/usr/bin/env julia
using RobotOS
@rosimport geometry_msgs.msg: Point, Pose, Pose2D, PoseStamped, Vector3, Twist
@rosimport nloptcontrol_planner.msg: Control
@rosimport nav_msgs.msg: Path

rostypegen()
using geometry_msgs.msg
using nloptcontrol_planner.msg
using nav_msgs.msg

import YAML

using VehicleModels
using NLOptControl
using MAVs
using PyCall

@pyimport tf.transformations as tf

"""
# used to publish the solution of the ocp to ROS params
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/28/2018, Last Modified: 3/10/2018 \n
--------------------------------------------------------------------------------------\n
"""
function setTrajParams(msg::Control)
  L = length(msg.t)

  if L > 0
    t = (); sa = (); vx = (); x = (); y = (); psi = ();
    for i in 1:L
      t = (t..., msg.t[i])
      x = (x..., msg.x[i])
      y = (y..., msg.y[i])
      vx = (vx..., msg.vx[i])
      sa = (sa..., msg.sa[i])
      psi = (psi..., msg.psi[i])
    end

    # update trajectory parameters
    plannerNamespace = RobotOS.get_param("system/nloptcontrol_planner/namespace")

    RobotOS.set_param(string(plannerNamespace,"/traj/t"),t)
    RobotOS.set_param(string(plannerNamespace,"/traj/x"),x)
    RobotOS.set_param(string(plannerNamespace,"/traj/yVal"),y)
    RobotOS.set_param(string(plannerNamespace,"/traj/vx"),vx)
    RobotOS.set_param(string(plannerNamespace,"/traj/sa"),sa)
    RobotOS.set_param(string(plannerNamespace,"/traj/psi"),psi)

  else
    error("L !> 0")
  end
  return nothing
end

"""
# used to set the obstacle data in the ocp
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 4/6/2017, Last Modified: 2/28/2018 \n
--------------------------------------------------------------------------------------\n
"""
function setObstacleData(params)

  if RobotOS.has_param("obstacle/radius")
    Q = params[2][7];           # number of obstacles the algorithm can handle

      r = deepcopy(RobotOS.get_param("obstacle/radius"))
      x = deepcopy(RobotOS.get_param("obstacle/x"))
      y = deepcopy(RobotOS.get_param("obstacle/y"))
      vx = deepcopy(RobotOS.get_param("obstacle/vx"))
      vy = deepcopy(RobotOS.get_param("obstacle/vy"))

      if isnan(r[1]) # initilized, no obstacles detected
        L = 0
      else
        L = length(r)               # number of obstacles detected
      end

      N = Q - L;
      if N < 0
        warn(" \n The number of obstacles detected exceeds the number of obstacles the algorithm was designed for! \n
                  Consider increasing the number of obstacles the algorithm can handle \n!")
      end

      for i in 1:Q
        if i <= L          # add obstacle
          setvalue(params[2][1][i],r[i]);
          setvalue(params[2][2][i],r[i]);
          setvalue(params[2][3][i],x[i]);
          setvalue(params[2][4][i],y[i]);
          setvalue(params[2][5][i],vx[i]);
          setvalue(params[2][6][i],vy[i]);
        else              # set non-detected obstacle off field
          setvalue(params[2][1][i],1.0);
          setvalue(params[2][2][i],1.0);
          setvalue(params[2][3][i],-100.0 - 3*i);
          setvalue(params[2][4][i],-100.0);
          setvalue(params[2][5][i],0.0);
          setvalue(params[2][6][i],0.0);
        end
      end
  end

  return nothing
end

"""
# in the case of a known environment
# at the begining of the simulation assign the obstacle information given in the YAML file
# to the ROS params for processed obstacle data
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/28/2018, Last Modified: 2/28/2018 \n
--------------------------------------------------------------------------------------\n
"""
function setInitObstacleParams(c)

  radius = c["obstacle"]["radius"]
  center_x = c["obstacle"]["x0"]
  center_y = c["obstacle"]["y0"]
  velocity_x = c["obstacle"]["vx"]
  velocity_y = c["obstacle"]["vy"]
  L = length(radius)
    r = (); x = (); y = (); vx = (); vy = ();
    for i in 1:L
      r = (r..., radius[i])
      x = (x..., center_x[i])
      y = (y..., center_y[i])
      vx = (vx..., velocity_x[i])
      vy = (vy..., velocity_y[i])
    end

  # initialize obstacle field parameters
  RobotOS.set_param("obstacle/radius",r)
  RobotOS.set_param("obstacle/x",x)
  RobotOS.set_param("obstacle/y",y)
  RobotOS.set_param("obstacle/vx",vx)
  RobotOS.set_param("obstacle/vy",vy)

  return nothing
end

"""
# publishs the current state of the vehicle to ROS params
# isequal(RobotOS.get_param("system/plant"),"3DOF")
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/28/2018, Last Modified: 2/28/2018 \n
--------------------------------------------------------------------------------------\n
"""
function setStateParams(n)

  X0 = zeros(n.ocp.state.num)
  # update using the current location of plant
  for st in 1:n.ocp.state.num
    X0[st] = n.r.ip.plant[n.ocp.state.name[st]][end]
  end

  RobotOS.set_param("state/x", X0[1])
  RobotOS.set_param("state/y", X0[2])
  RobotOS.set_param("state/sa", X0[3])
  RobotOS.set_param("state/r", X0[4])
  RobotOS.set_param("state/psi", X0[5])
  RobotOS.set_param("state/sa", X0[6])
  RobotOS.set_param("state/ux", X0[7])
  RobotOS.set_param("state/ax", X0[8])

  return nothing
end

"""
# at the begining of the simulation assign the initial state given in the YAML file
# to the initial state that will be updated again and again
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/28/2018, Last Modified: 2/28/2018 \n
--------------------------------------------------------------------------------------\n
"""
function setInitStateParams(c)
  if isequal(RobotOS.get_param("system/plant"),"3DOF")
    RobotOS.set_param("state/x", RobotOS.get_param("case/actual/X0/x"))
    RobotOS.set_param("state/y", RobotOS.get_param("case/actual/X0/yVal"))
    RobotOS.set_param("state/sa",RobotOS.get_param("case/actual/X0/sa"))
    RobotOS.set_param("state/r", RobotOS.get_param("case/actual/X0/r"))
    RobotOS.set_param("state/psi", RobotOS.get_param("case/actual/X0/psi"))
    RobotOS.set_param("state/sa", RobotOS.get_param("case/actual/X0/sa"))
    RobotOS.set_param("state/ux", RobotOS.get_param("case/actual/X0/ux"))
    RobotOS.set_param("state/ax", RobotOS.get_param("case/actual/X0/ax"))
  else
    RobotOS.set_param("state/chrono/x", RobotOS.get_param("case/actual/X0/x"))
    RobotOS.set_param("state/chrono/yVal", RobotOS.get_param("case/actual/X0/yVal"))
    RobotOS.set_param("state/chrono/sa",RobotOS.get_param("case/actual/X0/sa"))
    RobotOS.set_param("state/chrono/r", RobotOS.get_param("case/actual/X0/r"))
    RobotOS.set_param("state/chrono/psi", RobotOS.get_param("case/actual/X0/psi"))
    RobotOS.set_param("state/chrono/sa", RobotOS.get_param("case/actual/X0/sa"))
    RobotOS.set_param("state/chrono/ux", RobotOS.get_param("case/actual/X0/ux"))
    RobotOS.set_param("state/chrono/ax", RobotOS.get_param("case/actual/X0/ax"))
  end
  return nothing
end


"""
# used to update the initial state of the vehicle based off of ROS params
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/28/2018, Last Modified: 2/28/2018 \n
--------------------------------------------------------------------------------------\n
"""
function setStateData(n)

  # copy current vehicle state in case it changes
  x=deepcopy(RobotOS.get_param("state/chrono/x"))
  y=deepcopy(RobotOS.get_param("state/chrono/yVal"))
  v=deepcopy(RobotOS.get_param("state/chrono/sa"))
  r=deepcopy(RobotOS.get_param("state/chrono/r"))
  psi=deepcopy(RobotOS.get_param("state/chrono/psi"))
  sa=deepcopy(RobotOS.get_param("state/chrono/sa"))
  ux=deepcopy(RobotOS.get_param("state/chrono/ux"))
  ax=deepcopy(RobotOS.get_param("state/chrono/ax"))

  X0 = [x,y,v,r,psi,sa,ux,ax]
  updateX0!(n,X0)
  return nothing
end


"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 4/6/2017, Last Modified: 3/10/2018 \n
--------------------------------------------------------------------------------------\n
"""
function loop(pub,pub_path,n,c)

  init = false
  loop_rate = Rate(2.0) # 2 Hz
  while !is_shutdown()
      println("Running model for the: ",n.mpc.v.evalNum," time")

      # update optimization parameters based off of latest obstacle information
      if !RobotOS.get_param("system/nloptcontrol_planner/flags/known_environment")
        setObstacleData(n.ocp.params)
      end

      # update optimization parameters based off of latest vehicle state
    #  if ! isequal(RobotOS.get_param("system/plant"),"3DOF") # otherwise an external update on the initial state of the vehicle is needed
    #    setStateData(n)
    #  end  NOTE currently this is after the optimization, eventually put it just before

      updateAutoParams!(n)                        # update model parameters
      #status = autonomousControl!(n)                # rerun optimization
      status = optimize!(n)

      # advance time
      if isequal(RobotOS.get_param("system/plant"),"3DOF") # otherwise an external update on the initial state of the vehicle is needed
        #n.mpc.t0_actual = (n.mpc.v.evalNum-1)*n.mpc.tex  # NOTE this is for testing
        n.mpc.v.t = n.mpc.v.t + n.mpc.v.tex
        n.mpc.v.evalNum = n.mpc.v.evalNum + 1
      else
        #n.mpc.t0_actual = to_sec(get_rostime())
        n.mpc.v.t = to_sec(get_rostime())
        n.mpc.v.evalNum = n.mpc.v.evalNum + 1
      end

      msg = Control()
      #msg.t = n.mpc.t0_actual + n.r.ocp.tst
      msg.t = n.r.ocp.tst
      msg.x = n.r.ocp.X[:,1]
      msg.y = n.r.ocp.X[:,2]
      msg.psi = n.r.ocp.X[:,5]
      msg.sa = n.r.ocp.X[:,6]
      msg.vx = n.r.ocp.X[:,7]

      publish(pub, msg)

      path = Path()
      path.header.stamp = get_rostime()
      path.header.frame_id = "map" # TODO get from rosparams
      path.poses = Array{PoseStamped}(length(msg.t))
      for i in 1:length(msg.t)
        path.poses[i]= PoseStamped()
        path.poses[i].header.frame_id = "map"
        path.poses[i].header.stamp = get_rostime()
        path.poses[i].pose.position.x = msg.x[i]
        path.poses[i].pose.position.y = msg.y[i]
      end
      publish(pub_path, path)

      if isequal(RobotOS.get_param("system/plant"),"3DOF") # otherwise an external update on the initial state of the vehicle is needed
        sol, U = simIPlant!(n)      # simulating plant in VehicleModels.jl
        plant2dfs!(n,sol,U)
        setStateParams(n) # update X0 parameters in ROS
        if !n.s.mpc.predictX0 #  use the current known plant state to update OCP
          push!(n.r.ip.X0p,currentIPState(n))
        else
          predictX0!(n)
        end
        updateX0!(n) # update X0 in NLOptControl.jl
      else
        setStateData(n)    # update X0 in NLOptControl.jl based off of state/ parameters
      end

      if isequal(RobotOS.get_param("system/plant"),"3DOF")
        goalReached!(n)
        if isequal(n.f.mpc.goalReached,true)
          RobotOS.set_param("system/nloptcontrol_planner/flags/goal_attained",true)
          break
        end
      else
        # get chrono states, see if it is near goal
      end

      if !init  # calling this node initialized after the first solve so that /traj/ parameters are set
        init = true
        RobotOS.set_param("system/nloptcontrol_planner/flags/initialized",true)
        println("nloptcontrol_planner has been initialized.")
        while(RobotOS.get_param("system/flags/paused"))
        end
      end
      rossleep(loop_rate)  # sleep for leftover time
  end  # while()
end

"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 4/6/2017, Last Modified: 3/10/2018 \n
--------------------------------------------------------------------------------------\n
"""
function main()
  println("initializing nloptcontrol_planner node ...")
  init_node("nloptcontrol_planner")

  # message for solution to optimal control problem
  plannerNamespace = RobotOS.get_param("system/nloptcontrol_planner/namespace")
  pub = Publisher{Control}(string(plannerNamespace,"/control"), queue_size=10)
  pub_path = Publisher{Path}("/path", queue_size=10)

  sub = Subscriber{Control}(string(plannerNamespace, "/control"), setTrajParams, queue_size = 10)

  # using the filenames set as rosparams, the datatypes of the parameters get messed up if they are put on the ROS server
  # and then loaded into julia through RobotOS.jl; but less is messed up by loading using YAML.jl
  case = YAML.load(open(RobotOS.get_param("case_params_path")))["case"]
  planner = YAML.load(open(RobotOS.get_param("planner_params_path")))["planner"]["nloptcontrol_planner"]
  vehicle = YAML.load(open(RobotOS.get_param("vehicle_params_path")))["vehicle"]["nloptcontrol_planner"]

  c = YAML.load(open(string(Pkg.dir("MAVs"),"/config/empty.yaml")))
  c["vehicle"] = vehicle
  c["weights"] = planner["weights"]
  c["misc"] = planner["misc"]
  c["solver"] = planner["solver"]
  c["tolerances"] = planner["tolerances"]
  c["X0"] = case["actual"]["X0"]
  c["goal"] = case["goal"]

  if RobotOS.get_param("system/nloptcontrol_planner/flags/known_environment")
    c["obstacle"] = case["actual"]["obstacle"]
  else  # NOTE currently the the python parcer does not like assumed/obstacle format!, this will fail!
    c["obstacle"] = case["assumed"]["obstacle"]
  end
  fixYAML(c)   # fix messed up data types

  n = initializeAutonomousControl(c);
  setInitStateParams(c)

  if RobotOS.get_param("system/nloptcontrol_planner/flags/known_environment")
    setInitObstacleParams(c)
  end

  loop(pub,pub_path,n,c)
end

if !isinteractive()
    main()
end

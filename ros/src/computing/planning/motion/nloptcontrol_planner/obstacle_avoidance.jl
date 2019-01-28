#!/usr/bin/env julia
using RobotOS
@rosimport geometry_msgs.msg: Point, Pose, Pose2D, PoseStamped, Vector3, Twist
@rosimport nloptcontrol_planner.msg: Trajectory, Optimization
@rosimport nav_msgs.msg: Path
#@rosimport mavs_msgs.msg: state

rostypegen()
using geometry_msgs.msg
using nloptcontrol_planner.msg
using nav_msgs.msg
#using mavs_msgs.msg

import YAML
using NLOptControl
using MichiganAutonomousVehicles
using PyCall

@pyimport tf.transformations as tf

#          1  2  3  4  5    6   7   8
#names = [:x,:y,:v,:r,:psi,:sa,:ux,:ax];
#descriptions = ["X (m)","Y (m)","Lateral Velocity (m/s)", "Yaw Rate (rad/s)","Yaw Angle (rad)", "Steering Angle (rad)", "Longitudinal Velocity (m/s)", "Longitudinal Acceleration (m/s^2)"];

"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 11/4/2018, Last Modified: 11/4/2018 \n
--------------------------------------------------------------------------------------\n
"""
function goalAttained(xa,ya,xg,yg,gTol)
   return ((xa-xg)^2 + (ya-yg)^2)^0.5 < gTol
end

"""
# used to publish the solution of the ocp to ROS params
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/28/2018, Last Modified: 11/9/2018 \n
--------------------------------------------------------------------------------------\n
"""
function setTrajParams(msg::Trajectory)
  L = length(msg.t)

  if L > 0
    t = (); sa = (); ux = (); x = (); y = (); psi = ();
    v = (); r = (); ax = (); sr = (); jx = ();
    for i in 1:L
      t = (t..., msg.t[i])
      x = (x..., msg.x[i])
      y = (y..., msg.y[i])
      v = (v..., msg.v[i])
      r = (r..., msg.r[i])
      psi = (psi..., msg.psi[i])
      sa = (sa..., msg.sa[i])
      ux = (ux..., msg.ux[i])
      ax = (ax..., msg.ax[i])
      sr = (sr..., msg.sr[i])
      jx = (jx..., msg.jx[i])
    end

    # update trajectory parameters
    #plannerNamespace = RobotOS.get_param("system/nloptcontrol_planner/namespace")
    RobotOS.set_param(string("/trajectory/t"),t)
    RobotOS.set_param(string("/trajectory/x"),x)
    RobotOS.set_param(string("/trajectory/y"),y)
    RobotOS.set_param(string("/trajectory/v"),v)
    RobotOS.set_param(string("/trajectory/r"),r)
    RobotOS.set_param(string("/trajectory/psi"),psi)
    RobotOS.set_param(string("/trajectory/sa"),sa)
    RobotOS.set_param(string("/trajectory/ux"),ux)
    RobotOS.set_param(string("/trajectory/ax"),ax)
    RobotOS.set_param(string("/trajectory/sr"),sr)
    RobotOS.set_param(string("/trajectory/jx"),jx)
  else
    error("L !> 0")
  end
  return nothing
end

"""
Test Data:
r = [10,9,8,5,10,6,3]
x = [0,1,2,50,0,49,-30]
y = [0,0,2,50,0,49,-30]
vx = [0,0,0,0,0,0,0]
vy = [0,0,0,0,0,0,0]
L = 7

r,x,y,vx,vy = filterObstacleData(r,x,y,vx,vy,L)

# filter obstacle data to remove smallest overlapping circles
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 1/28/2019, Last Modified: 1/28/2019 \n
--------------------------------------------------------------------------------------\n
"""
function filterObstacleData(r,x,y,vx,vy,L)
    ra = (); xa = (); ya = (); vxa = (); vya = ();
    for i in 1:L # check each obstacle
        add = true
        # check all of the other obstacles
        for j in 1:L
            if 1!=j
                # intersection? && are you the smaller obstacle
                if (x[i] - x[j])^2 + (y[i] - y[j])^2 <= (r[i] + r[j])^2 && r[i] < r[j]
                    add = false
                    break
                end
            end
        end

        # check intersection with existing set
        if add
           for m in 1:length(ra)
               if (xa[m] - x[i])^2 + (ya[m] - y[i])^2 <= (ra[m] + r[i])^2
                   add = false
                   break
               end
           end
        end

        if add  # add the larger one to the set
            ra = (ra..., r[i])
            xa = (xa..., x[i])
            ya = (ya..., y[i])
            vxa = (vxa..., vx[i])
            vya = (vya..., vy[i])
        end
    end

    if length(ra) > L
        error("there is a bug in filterObstacleData(); it added obstacles to the set.")
    end
return ra, xa, ya, vxa, vya, length(ra)
end

"""
# used to set the obstacle data in the ocp
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 4/6/2017, Last Modified: 1/28/2019 \n
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
        r,x,y,vx,vy,L = filterObstacleData(r,x,y,vx,vy,length(r))
      end

      N = Q - L
      if N < 0
        warn("The number of obstacles detected exceeds the number of obstacles the algorithm was set to.")
      end

      for i in 1:Q
         # isequal(typeof(r[i]),Float64)
        if i <= L        # add obstacle
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
  RobotOS.set_param("state/v", X0[3])
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

    RobotOS.set_param("state/x", RobotOS.get_param("case/actual/X0/x"))
    RobotOS.set_param("state/y", RobotOS.get_param("case/actual/X0/yVal"))
    RobotOS.set_param("state/v",RobotOS.get_param("case/actual/X0/v"))
    RobotOS.set_param("state/r", RobotOS.get_param("case/actual/X0/r"))
    RobotOS.set_param("state/psi", RobotOS.get_param("case/actual/X0/psi"))
    RobotOS.set_param("state/sa", RobotOS.get_param("case/actual/X0/sa"))
    RobotOS.set_param("state/ux", RobotOS.get_param("case/actual/X0/ux"))
    RobotOS.set_param("state/ax", RobotOS.get_param("case/actual/X0/ax"))
    RobotOS.set_param("state/z",10.)
  return nothing
end


"""
# used to get current state of the vehicle
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/28/2018, Last Modified: 2/28/2018 \n
--------------------------------------------------------------------------------------\n
"""
function getStateData(n)

  # copy current vehicle state in case it changes
  x = deepcopy(RobotOS.get_param("state/x"))
  y = deepcopy(RobotOS.get_param("state/y"))
  v = deepcopy(RobotOS.get_param("state/v"))
  r = deepcopy(RobotOS.get_param("state/r"))
  psi = deepcopy(RobotOS.get_param("state/psi"))
  sa = deepcopy(RobotOS.get_param("state/sa"))
  ux = deepcopy(RobotOS.get_param("state/ux"))
  ax = deepcopy(RobotOS.get_param("state/ax"))
  return [x,y,v,r,psi,sa,ux,ax]
end


"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 4/6/2017, Last Modified: 11/9/2018 \n
--------------------------------------------------------------------------------------\n
"""
function loop(pub,pub_opt,pub_path,n,c)

  # define vectors to calculate moving average for tire forces
  MA = 4
  vtrrMA = zeros(MA)
  vtrlMA = zeros(MA)
  vtfrMA = zeros(MA)
  vtflMA = zeros(MA)
  tireCount = 1

  n.s.mpc.shiftX0 = true # tmp

  tSolveCum = 0
  tA = get_rostime()
  init = false
  tex = RobotOS.get_param("planner/nloptcontrol_planner/misc/tex")
  loop_rate = Rate(1/tex)
  while !is_shutdown()
      println("Running model for the: ",n.mpc.v.evalNum," time")

      # update optimization parameters based off of latest obstacle information
      if !RobotOS.get_param("system/nloptcontrol_planner/flags/known_environment")
        setObstacleData(n.ocp.params)
      end

      # update optimization parameters based off of latest vehicle state
    #  if ! isequal(RobotOS.get_param("system/plant"),"3DOF") # otherwise an external update on the initial state of the vehicle is needed
    #    setStateData(n)
    #  end  NOTE currently this is after the optimization, eventually put it just before for better performace.

      updateAutoParams!(n)                           # update model parameters
      optimize!(n)
    # @show n.r.ocp.status
    #  RobotOS.set_param("system/flags/initialized",true)
      if  init && n.r.ocp.status!=:Optimal && RobotOS.get_param("planner/nloptcontrol_planner/misc/onlyOptimal")
        RobotOS.set_param("system/flags/not_optimal",true)
        RobotOS.set_param("system/flags/done",true)
        println("The optimization was not optimally solved. Stopping simulation!")
        break
      end

      if isequal(RobotOS.get_param("system/plant"),"3DOF") # otherwise an external update on the initial state of the vehicle is needed
        n.mpc.v.t = n.mpc.v.t + n.mpc.v.tex
        n.mpc.v.evalNum = n.mpc.v.evalNum + 1
      else
        n.mpc.v.t = to_sec(get_rostime())
        n.mpc.v.evalNum = n.mpc.v.evalNum + 1
      end

      traj = Trajectory()
      traj.t = n.r.ocp.tst
      traj.x = n.r.ocp.X[:,1]
      traj.y = n.r.ocp.X[:,2]
      traj.v = n.r.ocp.X[:,3]
      traj.r = n.r.ocp.X[:,4]
      traj.psi = n.r.ocp.X[:,5]
      traj.sa = n.r.ocp.X[:,6]
      traj.ux = n.r.ocp.X[:,7]
      traj.ax = n.r.ocp.X[:,8]
      traj.sr = n.r.ocp.U[:,1]
      traj.jx = n.r.ocp.U[:,2]
      publish(pub, traj)

      opt = Optimization()
      opt.t =  get_rostime()
      opt.evalNum = n.mpc.v.evalNum - 1
      opt.texA = get_rostime() - tA
      opt.tSolve = n.r.ocp.tSolve
      opt.status = n.r.ocp.status

      # slow down optimization to real-time
      tSolveCum = tSolveCum + n.r.ocp.tSolve
      if init
        while(tSolveCum > Float64(get_rostime()))
        end
      end
      publish(pub_opt, opt)

      tA = get_rostime()

      path = Path()
      path.header.stamp = get_rostime()
      path.header.frame_id = "map" # TODO get from rosparams
      path.poses = Array{PoseStamped}(length(traj.t))
      for i in 1:length(traj.t)
        path.poses[i]= PoseStamped()
        path.poses[i].header.frame_id = "map"
        path.poses[i].header.stamp = get_rostime()
        path.poses[i].pose.position.x = traj.x[i]
        path.poses[i].pose.position.y = traj.y[i]
      end
      publish(pub_path, path)

      if isequal(RobotOS.get_param("system/plant"),"3DOF") # otherwise an external update on the initial state of the vehicle is needed
        sol, U = simIPlant!(n)      # simulating plant in VehicleModels.jl
        plant2dfs!(n,sol,U) # TODO see if this can be avoided
        setStateParams(n)           # update X0 parameters in
        updateX0!(n)                # update X0 in NLOptControl.jl
      else
         updateX0!(n,getStateData(n))
      end  # consider shifting to feasible.

      if isequal(RobotOS.get_param("system/plant"),"3DOF")
          xa = n.r.ip.plant[n.ocp.state.name[1]][end]
          ya = n.r.ip.plant[n.ocp.state.name[2]][end]
      else
          xa = deepcopy(RobotOS.get_param("state/x"))
          ya = deepcopy(RobotOS.get_param("state/y"))

          # in case vehicle falls off world in Chrono
          # TODO test this fall-off functionality
          z = deepcopy(RobotOS.get_param("state/z"))
    #      @show z
          if z < 0
            RobotOS.set_param("system/flags/fall",true)
            RobotOS.set_param("system/flags/done",true)
            println("The vehicle fell off the edge. Stopping simulation!")
            break
          end
          if Float64(get_rostime()) > 0.5
            vtrrMA[tireCount] = RobotOS.get_param("state/vtrr")
            vtrlMA[tireCount] = RobotOS.get_param("state/vtrl")
            vtfrMA[tireCount] = RobotOS.get_param("state/vtfr")
            vtflMA[tireCount] = RobotOS.get_param("state/vtfl")

            if Float64(get_rostime()) > 2 # only start to check after simulation has been running for at least 2 seconds
              #@show mean([mean(vtrlMA),mean(vtflMA)])
              #@show mean([mean(vtrrMA),mean(vtfrMA)])
              if isequal(mean([mean(vtrlMA),mean(vtflMA)]), 0.0) || isequal(mean([mean(vtrrMA),mean(vtfrMA)]), 0.0)
                RobotOS.set_param("system/flags/rollover",true)
                RobotOS.set_param("system/flags/done",true)
                println("The vehicle rolled over. Stopping simulation!")
                break
              end
            end

            tireCount = tireCount + 1
            if tireCount > MA
              tireCount = 1
            end
          end
      end

      if goalAttained(xa,ya,c["goal"]["x"],c["goal"]["yVal"],2*c["goal"]["tol"])
        RobotOS.set_param("system/flags/goal_attained",true)
        RobotOS.set_param("system/flags/done",true)
        println("The goal was attained. Stopping simulation!")
        break
      end

      if Float64(get_rostime()) > RobotOS.get_param("system/params/timelimit")
        RobotOS.set_param("system/flags/timelimit",true)
        RobotOS.set_param("system/flags/done",true)
        println("The simulation ran out of time. Stopping simulation!")
        break
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
  pub = Publisher{Trajectory}(string(plannerNamespace,"/control"), queue_size=10)
  pub_opt = Publisher{Optimization}(string(plannerNamespace,"/opt"), queue_size=10)
  pub_path = Publisher{Path}("/path", queue_size=10)

  sub = Subscriber{Trajectory}(string(plannerNamespace, "/control"), setTrajParams, queue_size = 10)

  #subRoll = Subscriber{state}("/state", setTireParams, queue_size = 10)
  # using the filenames set as rosparams, the datatypes of the parameters get messed up if they are put on the ROS server
  # and then loaded into julia through RobotOS.jl; but less is messed up by loading using YAML.jl
  case = YAML.load(open(RobotOS.get_param("case_params_path")))["case"]
  planner = YAML.load(open(RobotOS.get_param("planner_params_path")))["planner"]["nloptcontrol_planner"]
  vehicle = YAML.load(open(RobotOS.get_param("vehicle_params_path")))["vehicle"]["nloptcontrol_planner"]

  c = YAML.load(open(string(Pkg.dir("MichiganAutonomousVehicles"),"/config/empty.yaml")))
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

  loop(pub,pub_opt,pub_path,n,c)
end

if !isinteractive()
    main()
end

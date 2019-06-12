isdefined(Base, :__precompile__) && __precompile__()

module AutonomousControl

using NLOptControl
using RobotOS

include("CaseModule.jl")
using .CaseModule

include("VehicleModels/VehicleModels.jl")
using .VehicleModels

export
      initializeAutonomousControl,
      updateAutoParams!,
      solverConfig,
      avMpc,
      OptData,
      fixYAML,
      configProb!,    #TMP
      obstacleAvoidanceConstraints!,
      lidarConstraints!,
      objFunc!,
      goalRange

type OptData
      t
      sa
      ax
      obj
      X0
end
"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/15/2018, Last Modified: 3/12/2018 \n
-------------------------------------------------------------------------------------\n
"""
function solverConfig(c,init)

   if init      # initially giving solver more time
       maxtime = c["solver"]["maxtime_cpu_init"]
   else
       maxtime = c["solver"]["maxtime_cpu"]
   end

  # settings for both KNITRO and IPOPT
  outlev = (c["misc"]["solver"]==:Ipopt) ? :print_level : :outlev
  feastol_abs = (c["misc"]["solver"]==:Ipopt) ? :constr_viol_tol : :feastol_abs
  maxit = (c["misc"]["solver"]==:Ipopt) ? :max_iter : :maxit
  maxtime_cpu = (c["misc"]["solver"]==:Ipopt) ? :max_cpu_time : :maxtime_cpu
 if c["misc"]["solver"] == :KNITRO
   SS = ((:name=>c["misc"]["solver"]),(outlev=>c["solver"]["outlev"]),(feastol_abs=>c["solver"]["feastol_abs"]),(maxit=>c["solver"]["maxit"]),(maxtime_cpu=>maxtime),(maxit=>c["solver"]["maxit"]),(:ftol=>c["solver"]["ftol"]),(:feastol=>c["solver"]["feastol"]),(:ftol_iters=>c["solver"]["ftol_iters"]),(:infeastol=>c["solver"]["infeastol"]),(:maxfevals=>c["solver"]["maxfevals"]),(:opttol=>c["solver"]["opttol"]),(:opttol_abs=>c["solver"]["opttol_abs"]),(:xtol=>c["solver"]["xtol"]),(:algorithm=>c["solver"]["algorithm"]),(:bar_murule=>c["solver"]["bar_murule"]),(:linsolver=>c["solver"]["linsolver"]),(:cg_pmem=>c["solver"]["cg_pmem"]),(:bar_initpt=>c["solver"]["bar_initpt"]),(:bar_penaltycons=>c["solver"]["bar_penaltycons"]),(:bar_penaltyrule=>c["solver"]["bar_penaltyrule"]),(:bar_switchrule=>c["solver"]["bar_switchrule"]),(:linesearch=>c["solver"]["linesearch"]))
 elseif c["misc"]["solver"] == :Ipopt
   SS = ((:name=>c["misc"]["solver"]),(outlev=>c["solver"]["outlev"]),(feastol_abs=>c["solver"]["feastol_abs"]),(maxit=>c["solver"]["maxit"]),(maxtime_cpu=>maxtime),(maxit=>c["solver"]["maxit"]),(:acceptable_obj_change_tol=>c["solver"]["acceptable_obj_change_tol"]),(:warm_start_init_point=>c["solver"]["warm_start_init_point"]),(:dual_inf_tol=>c["solver"]["dual_inf_tol"]),(:compl_inf_tol=>c["solver"]["compl_inf_tol"]),(:acceptable_tol=>c["solver"]["acceptable_tol"]),(:acceptable_constr_viol_tol=>c["solver"]["acceptable_constr_viol_tol"]),(:acceptable_dual_inf_tol=>c["solver"]["acceptable_dual_inf_tol"]),(:acceptable_compl_inf_tol=>c["solver"]["acceptable_compl_inf_tol"]),(:acceptable_obj_change_tol=>c["solver"]["acceptable_obj_change_tol"]))
 end

return SS
end

"""
n = initializeAutonomousControl(c);
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/1/2017, Last Modified: 4/08/2018 \n
--------------------------------------------------------------------------------------\n
"""
function initializeAutonomousControl(c)

 pa = Vpara(ax_min=copy(c["vehicle"][:ax_min]),ax_max=copy(c["vehicle"][:ax_max]),m=copy(c["vehicle"][:m]),Izz=copy(c["vehicle"][:Izz]), la=copy(c["vehicle"][:la]), lb=copy(c["vehicle"][:lb]), FzF0=copy(c["vehicle"][:FzF0]), FzR0=copy(c["vehicle"][:FzR0]), KZX=copy(c["vehicle"][:KZX]), KZYR=copy(c["vehicle"][:KZYR]), KZYF=copy(c["vehicle"][:KZYF]), AXC=copy(c["vehicle"][:AXC]),x_min=copy(c["misc"]["Xmin"]),x_max=copy(c["misc"]["Xmax"]),y_min=copy(c["misc"]["Ymin"]),y_max=copy(c["misc"]["Ymax"]),sa_min=copy(c["vehicle"][:sa_min]),sa_max=copy(c["vehicle"][:sa_max]),psi_min=copy(c["vehicle"][:psi_min]),psi_max=copy(c["vehicle"][:psi_max]),u_min=copy(c["vehicle"][:u_min]),u_max=copy(c["vehicle"][:u_max]),sr_min=copy(c["vehicle"][:sr_min]),sr_max=copy(c["vehicle"][:sr_max]),jx_min=copy(c["vehicle"][:jx_min]),jx_max=copy(c["vehicle"][:jx_max]),FZ0=copy(c["vehicle"][:FZ0]),PCY1=copy(c["vehicle"][:PCY1]),PDY1=copy(c["vehicle"][:PDY1]),PDY2=copy(c["vehicle"][:PDY2]),PEY1=copy(c["vehicle"][:PEY1]),PEY2=copy(c["vehicle"][:PEY2]),PEY3=copy(c["vehicle"][:PEY3]),PKY1=copy(c["vehicle"][:PKY1]),PKY2=copy(c["vehicle"][:PKY2]),PHY1=copy(c["vehicle"][:PHY1]),PHY2=copy(c["vehicle"][:PHY2]),PVY1=copy(c["vehicle"][:PVY1]),PVY2=copy(c["vehicle"][:PVY2]),Caf=copy(c["vehicle"][:Caf]),Car=copy(c["vehicle"][:Car]),Fy_min=copy(c["vehicle"][:Fy_min]),Fy_max=copy(c["vehicle"][:Fy_max]),Fz_min=copy(c["vehicle"][:Fz_min]),Fz_off=copy(c["vehicle"][:Fz_off]),EP=copy(c["misc"]["EP"]))
 @unpack_Vpara pa # vehicle parameters

if isequal(c["misc"]["model"],:ThreeDOFv2)
    XF = [copy(c["goal"]["x"]), copy(c["goal"]["yVal"]), NaN, NaN, NaN, NaN, NaN, NaN]
    XL = [x_min, y_min, NaN, NaN, psi_min, sa_min, u_min, NaN]
    XU = [x_max, y_max, NaN, NaN, psi_max, sa_max, u_max, NaN]
    CL = [sr_min, jx_min]; CU = [sr_max, jx_max]
    X0 = [copy(c["X0"]["x"]),copy(c["X0"]["yVal"]),copy(c["X0"]["v"]),copy(c["X0"]["r"]),copy(c["X0"]["psi"]),copy(c["X0"]["sa"]),copy(c["X0"]["ux"]),copy(c["X0"]["ax"])]
    n = define(numStates=8,numControls=2,X0=copy(X0),XF=XF,XL=XL,XU=XU,CL=CL,CU=CU)
elseif isequal(c["misc"]["model"],:ThreeDOFv3)
    XF = [copy(c["goal"]["x"]), copy(c["goal"]["yVal"]), NaN, NaN, NaN, NaN]
    XL = [x_min, y_min, NaN, NaN, psi_min, sa_min]
    XU = [x_max, y_max, NaN, NaN, psi_max, sa_max]
    CL = [sr_min]; CU = [sr_max]
    X0 = [copy(c["X0"]["x"]),copy(c["X0"]["yVal"]),copy(c["X0"]["v"]),copy(c["X0"]["r"]),copy(c["X0"]["psi"]),copy(c["X0"]["sa"])]
    n = define(numStates=6,numControls=1,X0=copy(X0),XF=XF,XL=XL,XU=XU,CL=CL,CU=CU)
elseif isequal(c["misc"]["model"],:KinematicBicycle2)
    XF = [copy(c["goal"]["x"]), copy(c["goal"]["yVal"]),NaN,NaN]
    XL = [x_min,y_min,psi_min,u_min]
    XU = [x_max,y_max,psi_max,u_max]
    CL = [sa_min,ax_min]; CU = [sa_max,ax_max]
    X0 = [copy(c["X0"]["x"]),copy(c["X0"]["yVal"]),copy(c["X0"]["psi"]),copy(c["X0"]["ux"])]
    n = define(numStates=4,numControls=2,X0=copy(X0),XF=XF,XL=XL,XU=XU,CL=CL,CU=CU)
else
    error(string("c[misc][model] = " ,c["misc"]["model"]," needs to be set to either; :ThreeDOFv2 ||:ThreeDOFv3 || :KinematicBicycle2 "))
end
 n.s.ocp.tfMax = copy(c["misc"]["tfMax"])
 # need c for goalRange() in updateAutoParams!() which gets passed to simMpc!()
 n.ocp.params = [pa,NaN,NaN,NaN,c]

  # TODO check if Chrono is being used
if isequal(c["misc"]["model"],:ThreeDOFv2)

  #if isequal(c["misc"]["mode"], :OCP)
   IPModel = ThreeDOFv2
 # elseif isequal(c["misc"]["mode"], :IP)
#    IPModel = ThreeDOFv1
  #end

  # define tolerances on slack variables
  X0_tol = [c["tolerances"]["ix"], c["tolerances"]["iy"], c["tolerances"]["iv"], c["tolerances"]["ir"], c["tolerances"]["ipsi"], c["tolerances"]["isa"], c["tolerances"]["iu"], c["tolerances"]["iax"]]
  if goalRange(n)
    XF_tol = [c["tolerances"]["fx"], c["tolerances"]["fy"], c["tolerances"]["fv"], c["tolerances"]["fr"], c["tolerances"]["fpsi"], c["tolerances"]["fsa"], c["tolerances"]["fu"], c["tolerances"]["fax"]]
  else
    XF_tol = [1e6, 1e6, NaN, NaN, NaN, NaN, NaN, NaN]
  end
  defineTolerances!(n;X0_tol=X0_tol,XF_tol=XF_tol)

          # 1  2  3  4  5    6   7   8
  names = [:x,:y,:v,:r,:psi,:sa,:ux,:ax];
  descriptions = ["X (m)","Y (m)","Lateral Velocity (m/s)", "Yaw Rate (rad/s)","Yaw Angle (rad)", "Steering Angle (rad)", "Longitudinal Velocity (m/s)", "Longitudinal Acceleration (m/s^2)"];
  states!(n,names,descriptions=descriptions)

           # 1   2
  names = [:sr,:jx];
  descriptions = ["Steering Rate (rad/s)","Longitudinal Jerk (m/s^3)"];
  controls!(n,names,descriptions=descriptions)

  # dynamic constraints and additional constraints
  dx,con,tire_expr = ThreeDOFv2_expr(n)
  dynamics!(n,dx)
  constraints!(n,con)
elseif isequal(c["misc"]["model"],:ThreeDOFv3)

   #if isequal(c["misc"]["mode"], :OCP)
   IPModel = ThreeDOFv3
#   elseif isequal(c["misc"]["mode"], :IP)
#     IPModel = ThreeDOFv1
 #  end

   # define tolerances on slack variables
   X0_tol = [c["tolerances"]["ix"], c["tolerances"]["iy"], c["tolerances"]["iv"], c["tolerances"]["ir"], c["tolerances"]["ipsi"], c["tolerances"]["isa"]]
   if goalRange(n)
     XF_tol = [c["tolerances"]["fx"], c["tolerances"]["fy"], c["tolerances"]["fv"], c["tolerances"]["fr"], c["tolerances"]["fpsi"], c["tolerances"]["fsa"]]
   else
     XF_tol = [1e6, 1e6, NaN, NaN, NaN, NaN]
   end
   defineTolerances!(n;X0_tol=X0_tol,XF_tol=XF_tol)

           # 1  2  3  4  5    6
   names = [:x,:y,:v,:r,:psi,:sa];
   descriptions = ["X (m)","Y (m)","Lateral Velocity (m/s)", "Yaw Rate (rad/s)","Yaw Angle (rad)", "Steering Angle (rad)"];
   states!(n,names,descriptions=descriptions)

            # 1
   names = [:sr];
   descriptions = ["Steering Rate (rad/s)"];
   controls!(n,names,descriptions=descriptions)

   # dynamic constraints and additional constraints
   dx,con,tire_expr = ThreeDOFv3_expr(n)
   dynamics!(n,dx)
   constraints!(n,con)
elseif isequal(c["misc"]["model"],:KinematicBicycle2)
  # if isequal(c["misc"]["mode"], :OCP)
    IPModel = KinematicBicycle2
  # elseif isequal(c["misc"]["mode"], :IP)
#     IPModel = ThreeDOFv1
  # end
   # define tolerances on slack variables
   X0_tol = [c["tolerances"]["ix"], c["tolerances"]["iy"], c["tolerances"]["ipsi"], c["tolerances"]["iu"]]
   if goalRange(n)
    XF_tol = [c["tolerances"]["fx"], c["tolerances"]["fy"], c["tolerances"]["fpsi"], c["tolerances"]["fu"]]
   else
     XF_tol = [1e6,1e6,NaN,NaN]
   end
   defineTolerances!(n;X0_tol=X0_tol,XF_tol=XF_tol)

           # 1  2   3   4
   names = [:x,:y,:psi,:ux];
   descriptions = ["X (m)","Y (m)","Yaw Angle (rad)","Longitudinal Velocity (m/s)"];
   states!(n,names,descriptions=descriptions)

            # 1  2
   names = [:sa,:ax];
   descriptions = [ "Steering Angle (rad)","Longitudinal Acceleration (m/s^2)"];
   controls!(n,names,descriptions=descriptions)

   # dynamic constraints and additional constraints
   dx = KinematicBicycle_expr2(n)
   dynamics!(n,dx)
   tire_expr = NaN
 end
 configProb!(n,c)

 obs_params = obstacleAvoidanceConstraints!(n,c)
 LiDAR_params = lidarConstraints!(n,c)
 obj_params = objFunc!(n,c,tire_expr)
               #  1      2          3          4       5
 n.ocp.params = [pa,obs_params,LiDAR_params,obj_params,c]
 initOpt!(n;save=c["misc"]["ocpSave"],evalConstraints=c["misc"]["evalConstraints"])

 # after the initial optimization, reconfigure solver so that it does not have as much time as it needs
 defineSolver!(n,Dict(solverConfig(c,false)))

 # set mpc parameters
 if isequal(c["misc"]["model"],:ThreeDOFv2)
   goal = [c["goal"]["x"],c["goal"]["yVal"],NaN,NaN,NaN,NaN,NaN,NaN]
   goalTol = [c["goal"]["tol"],c["goal"]["tol"],NaN,NaN,NaN,NaN,NaN,NaN]
 elseif isequal(c["misc"]["model"],:ThreeDOFv3)
   goal = [c["goal"]["x"],c["goal"]["yVal"],NaN,NaN,NaN,NaN]
   goalTol = [c["goal"]["tol"],c["goal"]["tol"],NaN,NaN,NaN,NaN]
 elseif isequal(c["misc"]["model"],:KinematicBicycle2)
   goal = [c["goal"]["x"],c["goal"]["yVal"],NaN,NaN]
   goalTol = [c["goal"]["tol"],c["goal"]["tol"],NaN,NaN]
 end

 defineMPC!(n;onlyOptimal=c["misc"]["onlyOptimal"],mode=c["misc"]["mode"],goal=goal,goalTol=goalTol,fixedTp=c["misc"]["FixedTp"],predictX0=c["misc"]["PredictX0"],tp=c["misc"]["tp"],tex=copy(c["misc"]["tex"]),maxSim=copy(c["misc"]["mpc_max_iter"]))
 if isequal(c["misc"]["mode"], :OCP)
   defineIP!(n,IPModel)
 elseif isequal(c["misc"]["mode"], :IP) # NOTE for now assuming that the ThreeDOFv1 is always used for the IP
   error("TODO")
   defineIP!(n,IPModel)
 end

 return n
end

"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/20/2017, Last Modified: 3/12/2018 \n
--------------------------------------------------------------------------------------\n
"""
function updateAutoParams!(n)

  # obstacle information-> only show if it is in range at the start TODO
  goal_in_range = goalRange(n)
  if goal_in_range # TODO make a flag that indicates this switch has been flipped
    # otherwise it will keep setting the same parameters..
    println("goal is in range")
    c = n.ocp.params[5]

    # enforce final state constraints on x and y position
    # TODO consider throwing error if not using slack variables as they are not tested
    setRHS(n.r.ocp.xfsCon[1], c["tolerances"]["fx"])
    setRHS(n.r.ocp.xfsCon[2], c["tolerances"]["fy"])

    # add in slack variables in cost function for final state constraints
    setvalue(n.ocp.params[4][3],c["weights"]["xf"])

    # remove terms in cost function for line to goal
    setvalue(n.ocp.params[4][1],0.0)
    setvalue(n.ocp.params[4][2],0.0)

    # relax LiDAR constraints
    setvalue(n.ocp.params[3][1], 1e6)
    setvalue(n.ocp.params[3][2],-1e6)
  end
  #NOTE assuming it is not going in and out of range

 return goal_in_range
end


"""
# TODO use plant instead of mpc
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 7/04/2017, Last Modified: 3/12/2018 \n
--------------------------------------------------------------------------------------\n
"""
function goalRange(n)
  if isequal(n.r.ocp.evalNum,1)
    X = [n.ocp.X0[1],n.ocp.X0[2]]
  else
    X = currentIPState(n)[1]
  end
  c = n.ocp.params[5]
  return ( (X[1] - c["goal"]["x"])^2 + (X[2] - c["goal"]["yVal"])^2 )^0.5 < c["misc"]["Lr"]
end

"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/12/2018, Last Modified: 11/02/2018 \n
--------------------------------------------------------------------------------------\n
"""
function fixYAML(c)
   # need symbols to work with VehicleModels.jl
  c["vehicle"] = convert(Dict{Symbol,Any}, c["vehicle"])

  for keyA in ["weights", "misc", "X0", "solver", "tolerances"]
    for (key,value) in c[keyA]
      if isequal(value, "NaN")
        c[keyA][key] = NaN
      end
      if isequal(value, "true")
        c[keyA][key] = true
      end
      if isequal(value, "false")
        c[keyA][key] = false
      end
    end
  end

  for keyA in ["misc"]
    for (key,value) in c[keyA]
      if isequal(typeof(c[keyA][key]),String); c[keyA][key] = Symbol(c[keyA][key]); end
    end
  end

  return c
end

"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 4/08/2018, Last Modified: 1/25/2019 \n
--------------------------------------------------------------------------------------\n
"""
function configProb!(n,c)

  # configure problem
  if c["misc"]["integrationScheme"]==:lgrImplicit || c["misc"]["integrationScheme"]==:lgrExplicit
    configure!(n;(:xFslackVariables=>c["misc"]["xFslackVariables"]),(:x0slackVariables=>c["misc"]["x0slackVariables"]),(:Nck=>c["misc"]["Nck"]),(:integrationScheme=>c["misc"]["integrationScheme"]),(:finalTimeDV=>c["misc"]["finalTimeDV"]),(:solverSettings=>solverConfig(c,true)))
  else
    configure!(n;(:xFslackVariables=>c["misc"]["xFslackVariables"]),(:x0slackVariables=>c["misc"]["x0slackVariables"]),(:N=>c["misc"]["N"]),(:integrationScheme=>c["misc"]["integrationScheme"]),(:finalTimeDV=>c["misc"]["finalTimeDV"]),(:solverSettings=>solverConfig(c,true)))
  end
  return nothing
end

"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 4/08/2018, Last Modified: 4/08/2018 \n
--------------------------------------------------------------------------------------\n
"""
function obstacleAvoidanceConstraints!(n,c)

  rObs = copy(c["obstacle"]["radius"])
  xObs = copy(c["obstacle"]["x0"])
  yObs = copy(c["obstacle"]["y0"])
  vxObs = copy(c["obstacle"]["vx"])
  vyObs = copy(c["obstacle"]["vy"])

  Q = length(rObs); # number of obstacles
  @NLparameter(n.ocp.mdl, a[i=1:Q] == rObs[i])
  @NLparameter(n.ocp.mdl, b[i=1:Q] == rObs[i])
  @NLparameter(n.ocp.mdl, X_0[i=1:Q] == xObs[i])
  @NLparameter(n.ocp.mdl, Y_0[i=1:Q] == yObs[i])
  @NLparameter(n.ocp.mdl, speed_x[i=1:Q] == vxObs[i])
  @NLparameter(n.ocp.mdl, speed_y[i=1:Q] == vyObs[i])
  obs_params = [a,b,X_0,Y_0,speed_x,speed_y,Q]

  # obstacle postion after the initial postion
  if c["misc"]["movingObstacles"]
    X_obs = @NLexpression(n.ocp.mdl, [j=1:Q,i=1:n.ocp.state.pts], X_0[j] + speed_x[j]*n.ocp.tV[i])
    Y_obs = @NLexpression(n.ocp.mdl, [j=1:Q,i=1:n.ocp.state.pts], Y_0[j] + speed_y[j]*n.ocp.tV[i])
  else
    X_obs = @NLexpression(n.ocp.mdl, [j=1:Q,i=1:n.ocp.state.pts], X_0[j] + speed_x[j]*n.ocp.tV[1])
    Y_obs = @NLexpression(n.ocp.mdl, [j=1:Q,i=1:n.ocp.state.pts], Y_0[j] + speed_y[j]*n.ocp.tV[1])
  end

  # constraint on position
  x = n.r.ocp.x[:,1];y = n.r.ocp.x[:,2]; # pointers to JuMP variables

  sm = linspace(c["misc"]["sm1"],c["misc"]["sm2"],n.ocp.state.pts-1)
  obs_con = @NLconstraint(n.ocp.mdl, [j=1:Q,i=1:n.ocp.state.pts-1], 1 <= ((x[(i+1)]-X_obs[j,i])^2)/((a[j]+sm[i])^2) + ((y[(i+1)]-Y_obs[j,i])^2)/((b[j]+sm[i])^2))
  newConstraint!(n,obs_con,:obs_con);

  return obs_params
end

"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 4/08/2018, Last Modified: 4/08/2018 \n
--------------------------------------------------------------------------------------\n
"""
function lidarConstraints!(n,c)
  # ensure that the final x and y states are near the LiDAR boundary
  @NLparameter(n.ocp.mdl, LiDAR_param_1==(c["misc"]["Lr"] + c["misc"]["L_rd"])^2)
  @NLparameter(n.ocp.mdl, LiDAR_param_2==(c["misc"]["Lr"] - c["misc"]["L_rd"])^2)

  x = n.r.ocp.x[:,1];y = n.r.ocp.x[:,2]; # pointers to JuMP variables
  LiDAR_edge_high = @NLconstraint(n.ocp.mdl,[j=1], (x[end]-x[1])^2+(y[end]-y[1])^2  <= LiDAR_param_1)
  LiDAR_edge_low = @NLconstraint(n.ocp.mdl,[j=1], (x[end]-x[1])^2+(y[end]-y[1])^2  >= LiDAR_param_2)
  newConstraint!(n,LiDAR_edge_high,:LiDAR_edge_high)
  newConstraint!(n,LiDAR_edge_low,:LiDAR_edge_low)

 # constrain all state points to be within LiDAR boundary
  LiDAR_range = @NLconstraint(n.ocp.mdl, [j=1:n.ocp.state.pts-1], (x[j+1]-x[1])^2+(y[j+1]-y[1])^2 <= (c["misc"]["Lr"] + c["misc"]["L_rd"])^2 )

  newConstraint!(n,LiDAR_range,:LiDAR_range)

  if goalRange(n)   # relax LiDAR boundary constraints
     setvalue(LiDAR_param_1, 1e6)
     setvalue(LiDAR_param_2,-1e6)
  end
  LiDAR_params = [LiDAR_param_1,LiDAR_param_2]
  return LiDAR_params
end

"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 4/08/2018, Last Modified: 5/30/2010 \n
--------------------------------------------------------------------------------------\n
"""
function objFunc!(n,c,tire_expr)
  # parameters
  if goalRange(n)
   println("\n goal in range")
   @NLparameter(n.ocp.mdl, w_xf == c["weights"]["xf"])
   @NLparameter(n.ocp.mdl, w_goal_param == 0.0)
   @NLparameter(n.ocp.mdl, w_psi_param == 0.0)  # NOTE this param is not in use
  else
   @NLparameter(n.ocp.mdl, w_xf == 0.0)
   @NLparameter(n.ocp.mdl, w_goal_param == c["weights"]["goal"])
   @NLparameter(n.ocp.mdl, w_psi_param == c["weights"]["psi"])
  end

  obj_params = [w_goal_param,w_psi_param,w_xf]
  x = n.r.ocp.x[:,1];y = n.r.ocp.x[:,2]; # pointers to JuMP variables
  if isequal(c["misc"]["model"],:ThreeDOFv2)
    psi = n.r.ocp.x[:,5]
    sa = n.r.ocp.x[:,6]
    ax = n.r.ocp.x[:,8]
    sr = n.r.ocp.u[:,1]
    jx = n.r.ocp.u[:,2]
  elseif isequal(c["misc"]["model"],:ThreeDOFv3)
    psi = n.r.ocp.x[:,5]
    sa = n.r.ocp.x[:,6]
    sr = n.r.ocp.u[:,1]
  elseif isequal(c["misc"]["model"],:KinematicBicycle2)
    psi = n.r.ocp.x[:,3]
    sa = n.r.ocp.u[:,1]
    ax = n.r.ocp.u[:,2]
  end

  # penalize distance to goal
  goal_obj = @NLexpression(n.ocp.mdl,w_goal_param*((x[end] - c["goal"]["x"])^2 + (y[end] - c["goal"]["yVal"])^2)^0.5/(((x[1] - c["goal"]["x"])^2 + (y[1] - c["goal"]["yVal"])^2)^0.5 + c["misc"]["EP"]))

  # minimizing the integral over the entire prediction horizon of the line that passes through the goal
  haf_obj=integrate!(n,:( $c["weights"]["haf"]*( sin($c["goal"]["psi"])*(x[j]-$c["goal"]["x"]) - cos($c["goal"]["psi"])*(y[j]-$c["goal"]["yVal"]) )^2 ))

  if c["misc"]["xFslackVariables"]
    xf_obj = @NLexpression(n.ocp.mdl, w_xf*(n.ocp.xFs[1] + n.ocp.xFs[2]) )
  else
    xf_obj = 0
  end

  if isequal(c["misc"]["model"],:ThreeDOFv2)
    # initial conditions slack variables
    if c["misc"]["x0slackVariables"]
      x0_obj = @NLexpression(n.ocp.mdl, c["weights"]["ic"]*(c["weights"]["x0"]*(n.ocp.x0s[1]) + c["weights"]["y0"]*(n.ocp.x0s[2]) + c["weights"]["v0"]*(n.ocp.x0s[3]) + c["weights"]["r0"]*(n.ocp.x0s[4]) + c["weights"]["psi0"]*(n.ocp.x0s[5]) + c["weights"]["sa0"]*(n.ocp.x0s[6]) + c["weights"]["ux0"]*(n.ocp.x0s[7]) + c["weights"]["ax0"]*(n.ocp.x0s[8])) )
    else
      x0_obj = 0
    end

    # penalize difference between final heading angle and angle relative to the goal NOTE currently this is broken becasue atan2() is not available
    #psi_frg=@NLexpression(n.ocp.mdl,asin(c.g.y_ref-y[end])/(acos(c.g.x_ref-x[end]) + c["misc"]["EP"]) )
    #psi_obj=@NLexpression(n.ocp.mdl,w_psi_param*(asin(sin(psi[end] - psi_frg))/(acos(cos(psi[end] - psi_frg)) + c["misc"]["EP"]) )^2 )
    psi_obj = 0
    # psi_obj=@NLexpression(n.ocp.mdl,w_psi_param*(asin(sin(psi[end] - asin(c.g.y_ref-y[end])/acos(c.g.x_ref-x[end])))/(acos(cos(psi[end] - asin(c.g.y_ref-y[end])/acos(c.g.x_ref-x[end]))) + c["misc"]["EP"]) )^2 )

    # soft constraints on vertical tire load
    tire_obj = integrate!(n,tire_expr)

    # penalize control effort
    ce_obj = integrate!(n,:($c["weights"]["ce"]*($c["weights"]["sa"]*(sa[j]^2)+$c["weights"]["sr"]*(sr[j]^2)+$c["weights"]["jx"]*(jx[j]^2))) )
    @NLobjective(n.ocp.mdl, Min, xf_obj + goal_obj + x0_obj + psi_obj + c["weights"]["Fz"]*tire_obj + haf_obj + c["weights"]["time"]*n.ocp.tf + ce_obj )
  elseif isequal(c["misc"]["model"],:ThreeDOFv3)
      # initial conditions slack variables
      if c["misc"]["x0slackVariables"]
        x0_obj = @NLexpression(n.ocp.mdl, c["weights"]["ic"]*(c["weights"]["x0"]*(n.ocp.x0s[1]) + c["weights"]["y0"]*(n.ocp.x0s[2]) + c["weights"]["v0"]*(n.ocp.x0s[3]) + c["weights"]["r0"]*(n.ocp.x0s[4]) + c["weights"]["psi0"]*(n.ocp.x0s[5]) + c["weights"]["sa0"]*(n.ocp.x0s[6]) ) )
      else
        x0_obj = 0
      end
      psi_obj = 0
      # soft constraints on vertical tire load
      tire_obj = integrate!(n,tire_expr)

      # penalize control effort
      ce_obj = integrate!(n,:($c["weights"]["ce"]*($c["weights"]["sa"]*(sa[j]^2)+$c["weights"]["sr"]*(sr[j]^2))) )
      @NLobjective(n.ocp.mdl, Min, xf_obj + goal_obj + x0_obj + psi_obj + c["weights"]["Fz"]*tire_obj + haf_obj + c["weights"]["time"]*n.ocp.tf + ce_obj )

  elseif isequal(c["misc"]["model"],:KinematicBicycle2)
    # penalize control effort
    ce_obj = integrate!(n,:($c["weights"]["ce"]*($c["weights"]["sa"]*sa[j]^2 + $c["weights"]["ax"]*ax[j]^2 ) ) )
    @NLobjective(n.ocp.mdl, Min, xf_obj + goal_obj + c["weights"]["time"]*n.ocp.tf + ce_obj + haf_obj )
  else
    error("Please select use a model in the misc YAML params. Either ThreeDOFv2 or KinematicBicycle2.")
  end
 return obj_params
end



end # module

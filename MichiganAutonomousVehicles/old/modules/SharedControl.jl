module SharedControl

using MathProgBase # for checkFeasibility!

include("CaseModule.jl")
using .CaseModule

export
      initializeSharedControl!,
      checkFeasibility!,
      sharedControl!,
      getPlantData!,
      sendOptData!,
      ExternalModel

type ExternalModel  # communication
  s1
  s2
  status   # to pass result of optimization to Matlab
  sa_sample
  runJulia # a Bool (in Int form) to indicate to run julia comming from Matlab
  numObs   # number of obstacles
  SA       # last two drivers steering angle
  SA_first # first order approximation of drivers commands
  UX       # vehicle speed
  X0       # vehicle states
  Xobs
  Yobs
  A
  B
  t0  # current time
  t_opt
  wsa
  feasible # a bool for the feasibility of the current driver commands
  infeasible_counter
  infeasible_counter_max
  AL
  AU
  dt  # this is the time step in simulink
end

function ExternalModel()
  ExternalModel(Any,
                Any,
                1.0,
                [],
                1,
                3,
                [0.0,0.0],
                [],
                10.0,
                [],
                [],
                [],
                [],
                [],
                0.0,
                0.0,
                1.0,
                false,
                100,
                10,
                [],
                [],
                0.01)
end


"""
mdl,n,r,params,x,d=initializeSharedControl!(c)
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 1/27/2017, Last Modified: 4/4/2017 \n
--------------------------------------------------------------------------------------\n
"""
function initializeSharedControl!(c)
    if c.m.model!=:ThreeDOFv2; error(" this is for ThreeDOFv2 ! ") end

    if c.m.activeSafety
      pa=Vpara(x_min=c.m.Xlims[1],x_max=c.m.Xlims[2],y_min=c.m.Ylims[1],y_max=c.m.Ylims[2],sr_min=-0.18,sr_max=0.18);
    else
      pa=Vpara(x_min=c.m.Xlims[1],x_max=c.m.Xlims[2],y_min=c.m.Ylims[1],y_max=c.m.Ylims[2],sr_min=-0.10,sr_max=0.10); #0.12
    end
    @unpack_Vpara pa;

    XF=[  NaN, NaN,   NaN, NaN,     NaN,    NaN,    NaN, NaN];
  #  XL=[x_min, y_min, NaN, NaN, psi_min, sa_min, c.m.UX, 0.0];
  #  XU=[x_max, y_max, NaN, NaN, psi_max, sa_max, c.m.UX, 0.0];
    XL=[NaN, NaN, NaN, NaN, psi_min, sa_min, c.m.UX, 0.0];
    XU=[NaN, NaN, NaN, NaN, psi_max, sa_max, c.m.UX, 0.0];
    CL = [sr_min, 0.0]; CU = [sr_max, 0.0];
    n=define(ThreeDOFv2;numStates=8,numControls=2,X0=copy(c.m.X0),XF=XF,XL=XL,XU=XU,CL=CL,CU=CU);

    # variable names
             # 1  2  3  4  5    6   7   8
    names = [:x,:y,:v,:r,:psi,:sa,:ux,:ax];
    descriptions = ["X (m)","Y (m)","Lateral Velocity (m/s)", "Yaw Rate (rad/s)","Yaw Angle (rad)", "Steering Angle (rad)", "Longitudinal Velocity (m/s)", "Longitudinal Acceleration (m/s^2)"];
    stateNames!(n,names,descriptions);
             # 1    2
    names = [:sr,:jx];
    descriptions = ["Steering Rate (rad/s)","Longitudinal Jerk (m/s^3)"];
    controlNames!(n,names,descriptions);

    # configure problem
    configure!(n,Nck=c.m.Nck;(:integrationScheme => :lgrExplicit),(:finalTimeDV => false),(:tf => c.m.tp))
    mdl=defineSolver!(n,c);

    # define tolerances
    XF_tol=[NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN];
    X0_tol=[0.05,0.05,0.005,0.05,0.01,0.003,NaN,NaN];  # TODO BE CAREFUL HERE!!
    defineTolerances!(n;X0_tol=X0_tol,XF_tol=XF_tol);

    # add parameters
    @NLparameter(mdl, ux_param==c.m.UX);                       # inital vehicle speed
    @NLparameter(mdl, sa_param[i=1:n.numStatePoints]==0.0);    # driver steering command
    @NLparameter(mdl, sr_param==0.0);                          # driver steering rate command

    veh_params=[ux_param,sa_param,sr_param];

    # obstacles
    #Q=size(c.o.A)[1]; # number of obstacles TODO update these based off of LiDAR data
    Q=1;
    if Q>3; error("for activeSafety mode the code was modified to deal with only three obstales when checkFeasibility is running \n");end
    @NLparameter(mdl, a[i=1:Q] == c.o.A[i]);
    @NLparameter(mdl, b[i=1:Q] == c.o.B[i]);
    @NLparameter(mdl, X_0[i=1:Q] == c.o.X0[i]);
    @NLparameter(mdl, Y_0[i=1:Q] == c.o.Y0[i]);
    obs_params=[a,b,X_0,Y_0];

    x=ExternalModel(); # setup data with Matlab

    # setup mpc parameters
    initializeMPC!(n;FixedTp=c.m.FixedTp,PredictX0=c.m.PredictX0,tp=c.m.tp,tex=copy(c.m.tex),max_iter=c.m.mpc_max_iter);
    x.X0=copy(c.m.X0);

    # define ocp
    s=Settings(;save=false,MPC=true,evalConstraints=false);
    r=OCPdef!(mdl,n,s,[pa,ux_param]);  # need pa out of params -> also need speed for c.m.model==:ThreeDOFv1

    # define objective function
    obj=0;
    if c.m.followPath
      # follow the path -> min((X_path(Yt)-Xt)^2)
      if c.t.func==:poly
        path_obj=@NLexpression(mdl,c.w.path*sum(  (  (c.t.a[1] + c.t.a[2]*r.x[(i+1),2] + c.t.a[3]*r.x[(i+1),2]^2 + c.t.a[4]*r.x[(i+1),2]^3 + c.t.a[5]*r.x[(i+1),2]^4) - r.x[(i+1),1]  )^2 for i in 1:n.numStatePoints-1)  );
      elseif c.t.func==:fourier
        path_obj=@NLexpression(mdl,c.w.path*sum(  (  (c.t.a[1]*sin(c.t.b[1]*r.x[(i+1),1]+c.t.c[1]) + c.t.a[2]*sin(c.t.b[2]*r.x[(i+1),1]+c.t.c[2]) + c.t.a[3]*sin(c.t.b[3]*r.x[(i+1),1]+c.t.c[3]) + c.t.a[4]*sin(c.t.b[4]*r.x[(i+1),1]+c.t.c[4]) + c.t.a[5]*sin(c.t.b[5]*r.x[(i+1),1]+c.t.c[5]) + c.t.a[6]*sin(c.t.b[6]*r.x[(i+1),1]+c.t.c[6]) + c.t.a[7]*sin(c.t.b[7]*r.x[(i+1),1]+c.t.c[7]) + c.t.a[8]*sin(c.t.b[8]*r.x[(i+1),1]+c.t.c[8])+c.t.y0) - r.x[(i+1),2]  )^2 for i in 1:n.numStatePoints-1)  );
      end
      obj=path_obj;
    end

    if c.m.followDriver
       driver_obj=integrate!(mdl,n,r.x[:,6];C=c.w.driver,D=copy(sa_param),(:variable=>:control),(:integrand=>:squared),(:integrandAlgebra=>:subtract));
       obj=@NLexpression(mdl,obj+driver_obj);
    end
    sr_obj=integrate!(mdl,n,r.u[:,1];C=c.w.sr,(:variable=>:control),(:integrand=>:squared));     # minimize steering rate

    @NLobjective(mdl, Min, obj+sr_obj);

    # constraint steering rate
    sr_tol=0.005; #0.01
    sr_conL=@NLconstraint(mdl, [i=1],r.u[i,1] <=  (sr_param+sr_tol));
    newConstraint!(r,sr_conL,:sr_conL);
    sr_conU=@NLconstraint(mdl, [i=1],-r.u[i,1] <= -(sr_param-sr_tol));
    newConstraint!(r,sr_conU,:sr_conU);

    #sr_con=@NLconstraint(mdl,[i=1],r.u[i,1]==sr_param);
    #newConstraint!(r,sr_con,:sr_con);

    # obstacle postion after the intial postion
    @NLexpression(mdl, X_obs[j=1:Q,i=1:n.numStatePoints], X_0[j])
    @NLexpression(mdl, Y_obs[j=1:Q,i=1:n.numStatePoints], Y_0[j])

    # constraint position
    j=1;obs_con1=@NLconstraint(mdl, [i=3:n.numStatePoints-1], 1 <= ((r.x[(i+1),1]-X_obs[j,i])^2)/((a[j]+c.m.sm)^2) + ((r.x[(i+1),2]-Y_obs[j,i])^2)/((b[j]+c.m.sm)^2));
    newConstraint!(r,obs_con1,:obs_con1);
    #j=2;obs_con2=@NLconstraint(mdl, [i=1:n.numStatePoints-1], 1 <= ((r.x[(i+1),1]-X_obs[j,i])^2)/((a[j]+c.m.sm)^2) + ((r.x[(i+1),2]-Y_obs[j,i])^2)/((b[j]+c.m.sm)^2));
    #newConstraint!(r,obs_con2,:obs_con2);
    #j=3;obs_con3=@NLconstraint(mdl, [i=1:n.numStatePoints-1], 1 <= ((r.x[(i+1),1]-X_obs[j,i])^2)/((a[j]+c.m.sm)^2) + ((r.x[(i+1),2]-Y_obs[j,i])^2)/((b[j]+c.m.sm)^2));
    #newConstraint!(r,obs_con3,:obs_con3);

    # LiDAR connstraint  TODO finish fixing the lidar constraints here NOTE currently not using this
    @NLparameter(mdl, X0_params[j=1:2]==n.X0[j]);

  #  LiDAR_con=@NLconstraint(mdl, [i=1:n.numStatePoints-1], ((r.x[(i+1),1]-X0_params[1])^2+(r.x[(i+1),2]-X0_params[2])^2) <= (c.m.Lr + c.m.L_rd)^2); # not constraining the first state
  #  newConstraint(r,LiDAR_con,:LiDAR_con);
          #   1-3      4

    # constraint on progress on track (no turning around!)
    #=
    if c.t.dir==:posY
      progress_con=@NLconstraint(mdl, [i=1:n.numStatePoints-1], r.x[i,2] <= r.x[(i+1),2]);
      newConstraint!(r,progress_con,:progress_con);
    elseif c.t.dir==:posX
      progress_con=@NLconstraint(mdl, [i=1:n.numStatePoints-1], r.x[i,1] <= r.x[(i+1),1]);
      newConstraint!(r,progress_con,:progress_con);
    end
=#
    # intial optimization
    optimize!(mdl,n,r,s);

    if c.m.activeSafety
      evalConstraints!(n,r);         # setup constraint data
      d=JuMP.NLPEvaluator(mdl);      # initialize feasibility checking functionality
      MathProgBase.initialize(d,[:Grad]);
      checkFeasibility!(mdl,d,n,c,x,pa,r;feas_tol=0.005);
    else
      d=NaN
    end
          #  1      2          3         4
    params=[pa,veh_params, obs_params,X0_params];
    return mdl,n,r,params,x,d
end
"""

checkFeasibility!(mdl,d,n,c,x,params[1],r;feas_tol=0.005);
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/13/2017, Last Modified: 4/11/2017 \n
--------------------------------------------------------------------------------------\n
"""
function checkFeasibility!(mdl,d,n,c,x,pa,r;feas_tol::Float64=0.005)

  # simulate vehicle response
  U=zeros(n.numStatePoints,2); # NOTE using numStatePoints so that we can approximate the state at the end point
  SR=copy((x.SA[1]-x.SA[2])/x.dt)*ones(n.numStatePoints,);  # first order approximation of SR
  #JX=0.0;
  U[:,1]=SR;
  #U[:,2]=JX; #NOTE for now assue a constant steering angle
  t0=copy(x.t0); tf=t0+copy(c.m.tp); # tf should be the same as t[end]
  t=t0+r.t_st; # for a fixed final time, this never changes
  sol=simModel(n,pa,x.X0,t,U,t0,tf);

  # pass values to constraint equations
  num=n.numStatePoints*n.numStates + n.numControlPoints*n.numControls; # NOTE we removed t0 as a design variable
  values=zeros(num);
  for st in 1:n.numStates
    for j in 1:n.numStatePoints
     values[linearindex(r.x[j,st])] = sol(t[j])[st];
    end
  end
  #idx=1; #NOTE for now assue a constant steering angle
  #for j in 1:n.numControlPoints;  # assuming JX is zero
  #  values[linearindex(r.u[idx,1])] = SR[idx]; idx+=1;
  #end
  g=zeros(MathProgBase.numconstr(d.m));; # TODO fix -->number of constraints is different from num TODO remove that if we are not doing mpc??
  MathProgBase.eval_g(d,g,values)# is this broken ?
  b=JuMP.constraintbounds(mdl);
  AL=g-b[1]-feas_tol; AU=b[2]+feas_tol-g; # give tolerance

  # check obstacle avoidance constraints
  L1=0;U1=0;  #L2=0;U2=0;  L3=0;U3=0;

  for i in 1:length(r.constraint.name) # note only setup for three obstacles currently
    if r.constraint.name[i]==:obs_con1
      L1=r.constraint.nums[i][end][1];
      U1=r.constraint.nums[i][end][2]-c.m.NF;
    end

#=
    if r.constraint.name[i]==:obs_con2
      L2=r.constraint.nums[i][end][1];
      U2=r.constraint.nums[i][end][2]-c.m.NF;
    end
    if r.constraint.name[i]==:obs_con3
      L3=r.constraint.nums[i][end][1];
      U3=r.constraint.nums[i][end][2]-c.m.NF;
    end
    =#
  end
  #x.AL=[AL[L1:U1];AL[L2:U2];AL[L3:U3]];
#  x.AU=[AU[L1:U1];AU[L2:U2];AU[L3:U3]];
  x.AL=AL[L1:U1];
  x.AU=AU[L1:U1];
  if minimum(sign([x.AL;x.AU]))==-1;x.feasible=false;else;x.feasible=true; end
  nothing
end

"""

--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 4/6/2017, Last Modified: 4/7/2017 \n
--------------------------------------------------------------------------------------\n
"""
function getPlantData!(n,params,x,c,r)
  MsgString = recv(x.s1);L=19;
  MsgIn = zeros(L);
  allidx = find(MsgString->MsgString == 0x20,MsgString);
  allidx = [0;allidx];
  for i in 1:L  #length(allidx)-1
    MsgIn[i] = parse(Float64,String(copy(MsgString[allidx[i]+1:allidx[i+1]-1])));
  end
  x.X0=zeros(n.numStates);
  x.X0[1:7]=MsgIn[1:7];              # global X, global y, lateral speed v, yaw rate r, yaw angle psi
  x.SA=zeros(2);
  x.SA=[copy(x.X0[6]),MsgIn[8]];     # Vehicle steering angle and previous state of steering angle
  x.UX=copy(x.X0[7]);

  N=n.numStates+1;
  x.Xobs=MsgIn[N:N+x.numObs-1];               # ObsX
  x.Yobs=MsgIn[N+x.numObs:N+2*x.numObs-1];    # ObsY
  x.A= MsgIn[N+2*x.numObs:N+3*x.numObs-1];    # ObsR
  x.B=x.A;
  x.runJulia=MsgIn[N+3*x.numObs];
  x.t0=MsgIn[N+3*x.numObs+1];
  n.mpc.t0_actual=copy(x.t0);

  # update drivers steering rate
  SR=copy((x.SA[1]-x.SA[2])/x.dt);
  if SR>0.086;SR=0.086;elseif SR<-0.086;SR=0.086;end # saturate SR
  setvalue(params[2][3],SR)

  # update obstacle feild
#  for i in 1:length(x.A)
  i=1;
  setvalue(params[3][1][i],x.A[i]);
  setvalue(params[3][2][i],x.B[i]);
  setvalue(params[3][3][i],x.Xobs[i]);
  setvalue(params[3][4][i],x.Yobs[i]);
#  end

  # update case for plotting
  if r.eval_num==1; c.o.A=[];c.o.B=[];c.o.X0=[];c.o.Y0=[]end # reset

  if isempty(c.o.X0) # update obstacle data
    push!(c.o.A,copy(x.A[1]));
    push!(c.o.B,copy(x.B[1]));
    push!(c.o.X0,copy(x.Xobs[1]));
    push!(c.o.Y0,copy(x.Yobs[1]));
    push!(c.o.s_y,0.0);
    push!(c.o.s_x,0.0);
  elseif  (x.Xobs!=c.o.X0[end]) && (x.Yobs!=c.o.Y0[end])
    push!(c.o.A,copy(x.A[1]));
    push!(c.o.B,copy(x.B[1]));
    push!(c.o.X0,copy(x.Xobs[1]));
    push!(c.o.Y0,copy(x.Yobs[1]));
    push!(c.o.s_y,0.0);
    push!(c.o.s_x,0.0);
  end

  x.SA_first=zeros(n.numStatePoints,);
  for jj in 1:n.numStatePoints
    x.SA_first[jj]=copy(x.SA[1]) + jj*copy(x.SA[1]-x.SA[2]);  # first order approximation of SA
  end

  if c.m.followDriver # first order
    for jj in 1:n.numStatePoints
      setvalue(params[2][2][jj], x.SA_first[jj])
    end
  end
  nothing
end

"""

--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 4/6/2017, Last Modified: 4/7/2017 \n
--------------------------------------------------------------------------------------\n
"""
function sendOptData!(n,r,x)
  if !x.feasible || (x.infeasible_counter < x.infeasible_counter_max)
    println("Sending Optimized Steering Angle Commands to MATLAB");
    if r.dfs_opt[r.eval_num-1][:status][1]==:Optimal # if infeasible -> let user control -> the 3 is a flag in Matlab that tells the system not to use signals from Autonomy
      MsgOut = [x.sa_sample;r.dfs_opt[r.eval_num-1][:t_solve];2;1]
    else
      MsgOut = [x.sa_sample;r.dfs_opt[r.eval_num-1][:t_solve];3;1];
    end
    # send UDP packets to client side
    TJ=copy(n.mpc.t0_actual)
    MsgOut = [MsgOut;Float64(r.eval_num-1);TJ];
  else
    MsgOut=[copy(x.SA[1])*ones(31);0.01;1;1]
  end
  MsgOutString = ' ';
  for j in 1:length(MsgOut)
      MsgOutString = string(MsgOutString,' ',MsgOut[j]);
  end
  MsgOutString = string(MsgOutString," \n");
  send(x.s2,ip"141.212.141.245",36881,MsgOutString);  # change this to the ip where you are running Simulink!
  nothing
end
"""

--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 1/27/2017, Last Modified: 4/7/2017 \n
--------------------------------------------------------------------------------------\n
"""
function sharedControl!(mdl,n,r,s,params,x)
  # rerun optimization
  status=autonomousControl!(mdl,n,r,s,params[1]);

  # sample solution
  sp_SA=Linear_Spline(r.t_st,r.X[:,6]);
  t_sample=Ranges.linspace(0.0,3.0,31);
  x.sa_sample=sp_SA[t_sample];

  # update status for Matlab
  if status!=:Infeasible; x.status=1.; else x.status=0.; end
  nothing
end

"""
evalNum()
# to extract data from a particular iteration number
# will not work unless the data was saved; i.e., s.save = true
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/21/2017, Last Modified: 2/21/2017 \n
--------------------------------------------------------------------------------------\n
"""
function evalNum(Idx)
  eval_num=0;
  for i in 1:length(r.dfs_opt)
    if r.dfs_opt[i][:iter_num][1]==Idx
      eval_num=i;
      break
    end
  end
  print(eval_num);
  s=Settings(;format=:png,MPC=false);
  cd("results/test1"); allPlots(n,r,s,eval_num); cd(main_dir)
end



end # module

module PathFollowing

using NLOptControl
using VehicleModels

include("CaseModule.jl")
using .CaseModule

export
      initializePathFollowing,
      updatePathParams!

"""

--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/28/2017, Last Modified: 4/3/2017 \n
--------------------------------------------------------------------------------------\n
"""
function initializePathFollowing(c)  #TODO add in some sort of a window around the vehicle where it only sees certain obstacles
  pa=Vpara(x_min=c.m.Xlims[1],x_max=c.m.Xlims[2],y_min=c.m.Ylims[1],y_max=c.m.Ylims[2],sr_min=-0.18,sr_max=0.18);
  @unpack_Vpara pa;

  XF=[  NaN, NaN,   NaN, NaN,     NaN,    NaN,    NaN, NaN];
  XL=[NaN,NaN, NaN, NaN, psi_min, sa_min, c.m.UX, 0.0];
  XU=[NaN,NaN, NaN, NaN, psi_max, sa_max, c.m.UX, 0.0];
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
  @NLparameter(mdl, ux_param==c.m.UX); # inital vehicle speed

  # obstacles
  Q = size(c.o.A)[1]; # number of obstacles TODO update these based off of LiDAR data
  @NLparameter(mdl, a[i=1:Q] == copy(c.o.A[i]));
  @NLparameter(mdl, b[i=1:Q] == copy(c.o.B[i]));
  @NLparameter(mdl, X_0[i=1:Q] == copy(c.o.X0[i]));
  @NLparameter(mdl, Y_0[i=1:Q] == copy(c.o.Y0[i]));
  obs_params=[a,b,X_0,Y_0];

  # set mpc parameters
  initializeMPC!(n;FixedTp=c.m.FixedTp,PredictX0=c.m.PredictX0,tp=c.m.tp,tex=copy(c.m.tex),max_iter=c.m.mpc_max_iter);
  n.mpc.X0=[copy(c.m.X0)];

  # define ocp
  s=Settings(;save=false,MPC=true);
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

  sr_obj=integrate!(mdl,n,r.u[:,1];C=c.w.sr,(:variable=>:control),(:integrand=>:squared));
  @NLobjective(mdl,Min,obj+sr_obj);

  # obstacle postion after the intial postion
  @NLexpression(mdl, X_obs[j=1:Q,i=1:n.numStatePoints], X_0[j])
  @NLexpression(mdl, Y_obs[j=1:Q,i=1:n.numStatePoints], Y_0[j])

  # constraint position
  obs_con=@NLconstraint(mdl, [j=1:Q,i=1:n.numStatePoints-1], 1 <= ((r.x[(i+1),1]-X_obs[j,i])^2)/((a[j]+c.m.sm)^2) + ((r.x[(i+1),2]-Y_obs[j,i])^2)/((b[j]+c.m.sm)^2));
  newConstraint!(r,obs_con,:obs_con);

  # LiDAR connstraint  TODO finish fixing the lidar constraints here
  @NLparameter(mdl, X0_params[j=1:2]==n.X0[j]);

#  LiDAR_con=@NLconstraint(mdl, [i=1:n.numStatePoints-1], ((r.x[(i+1),1]-X0_params[1])^2+(r.x[(i+1),2]-X0_params[2])^2) <= (c.m.Lr + c.m.L_rd)^2); # not constraining the first state
#  newConstraint(r,LiDAR_con,:LiDAR_con);
        #   1-3      4

  # constraint on progress on track (no turning around!)
  if c.t.dir==:posY
    progress_con=@NLconstraint(mdl, [i=1:n.numStatePoints-1], r.x[i,2] <= r.x[(i+1),2]);
    newConstraint!(r,progress_con,:progress_con);
  elseif c.t.dir==:posX
    progress_con=@NLconstraint(mdl, [i=1:n.numStatePoints-1], r.x[i,1] <= r.x[(i+1),1]);
    newConstraint!(r,progress_con,:progress_con);
  end

  # intial optimization
  optimize!(mdl,n,r,s);

        #  1    2          3         4
  n.params=[pa,ux_param,obs_params,X0_params];

  return mdl,n,r,params
end

"""
# called after updateX0 and before optimization
# TODO move this and other things like it to utils

# careful, when you go back to shared control make sure that you remove this from NLOPT
setvalue(params[3], SA)   # update desired steering angle
setvalue(params[2], UX)   # update speed
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/20/2017, Last Modified: 3/21/2017 \n
--------------------------------------------------------------------------------------\n
"""


function updatePathParams!(n,r,c,params)

  # vehicle position for LiDAR
  setvalue(params[4][1],n.X0[1])
  setvalue(params[4][2],n.X0[2])

  # obstacle information


  if c.m.model==:ThreeDOFv1

  elseif c.m.model==:ThreeDOFv2

  else
    error("\n set c.m.model \n")
  end

  nothing
end


end

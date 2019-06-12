using NLOptControl, VehicleModels, PrettyPlots, JuMP, Plots, Ipopt, DataFrames, Parameters
include("setup.jl")
include("DegeneracyHunter.jl")

function nlp1()
  pa=Vpara(x_min=185.,y_min=-0.1);
  n=NLOpt(); @unpack_Vpara pa
  X0=[  x0_,   y0_, v0_, r0_,   psi0_,   sa0_,  u0_ , ax0_];
  XF=[x_ref, y_ref, NaN, NaN,     NaN,    NaN,   u0_,  NaN];
  XL=[x_min, y_min, NaN, NaN, psi_min, sa_min, u_min, NaN];
  XU=[x_max, y_max, NaN, NaN, psi_max, sa_max, u_max, NaN];
  CL = [sr_min, jx_min]; CU = [sr_max, jx_max];
  n = define(n,stateEquations=ThreeDOFv2,numStates=8,numControls=2,X0=X0,XF=XF,XL=XL,XU=XU,CL=CL,CU=CU,tf_max=40.0)
  n = configure(n,Ni=2,Nck=[10,10];(:integrationMethod => :ps),(:integrationScheme => :lgrExplicit),(:finalTimeDV => true))
  #n = configure(n,N=60;(:integrationMethod => :tm),(:integrationScheme => :bkwEuler),(:finalTimeDV => true))
  #n = configure(n,N=60;(:integrationMethod => :tm),(:integrationScheme => :trapezoidal),(:finalTimeDV => true))
  mdl     = Model(solver = IpoptSolver(print_level=0,
                                       max_iter=500,
                                       tol=1e-1,
                                       dual_inf_tol=1100.,
                                       constr_viol_tol=1e-1,
                                       compl_inf_tol=1.,
                                       acceptable_tol=1e-3,
                                       acceptable_constr_viol_tol=0.01,
                                       acceptable_dual_inf_tol=1e10,
                                       acceptable_compl_inf_tol=0.01,
                                       acceptable_obj_change_tol=1e20,
                                       diverging_iterates_tol=1e20
                                       )
                      )
  names = [:x,:y,:v,:r,:psi,:sa,:ux,:ax];
  descriptions = ["X (m)","Y (m)","Lateral Velocity (m/s)", "Yaw Rate (rad/s)","Yaw Angle (rad)", "Steering Angle (rad)", "Longitudinal Velocity (m/s)", "Longitudinal Acceleration (m/s^2)"];
  stateNames(n,names,descriptions)
  names = [:sr,:jx];
  descriptions = ["Steering Rate (rad/s)","Longitudinal Jerk (m/s^3)"];
  controlNames(n,names,descriptions)
  params = [pa];   # vehicle parameters

  # define tolerances
  #XF_tol=[0.5,0.5,NaN,NaN,NaN,NaN,0.5,NaN];
  #X0_tol=[0.05,0.05,0.05,0.05,0.1,0.001,0.05,0.05];
  #defineTolerances(n;X0_tol=X0_tol,XF_tol=XF_tol);
  n,r = OCPdef(mdl,n,params);
  sr_obj = integrate(mdl,n,r.u[:,1];C=w_sr,(:variable=>:control),(:integrand=>:squared))
  @NLobjective(mdl, Min, n.tf + sr_obj)

  # obstacles
  Q = size(A)[1]; # number of obstacles
  @NLparameter(mdl, a[i=1:Q] == A[i]);
  @NLparameter(mdl, b[i=1:Q] == B[i]);
  @NLparameter(mdl, X_0[i=1:Q] == X0_obs[i]);
  @NLparameter(mdl, Y_0[i=1:Q] == Y0_obs[i]);

  # obstacle postion after the intial postion
  X_obs=@NLexpression(mdl, [j=1:Q,i=1:n.numStatePoints], X_0[j])
  Y_obs=@NLexpression(mdl, [j=1:Q,i=1:n.numStatePoints], Y_0[j])

  # constraint on position
  obs_con=@NLconstraint(mdl, [j=1:Q,i=1:n.numStatePoints-1], 1 <= ((r.x[(i+1),1]-X_obs[j,(i+1)])^2)/((a[j]+sm)^2) + ((r.x[(i+1),2]-Y_obs[j,(i+1)])^2)/((b[j]+sm)^2))
  newConstraint(r,obs_con,:obs_con);
  # TODO  make y0_ and x0_  JuMP parameters
  # ensure that all of the points are in the LiDAR range
  LiDAR_con=@NLconstraint(mdl, [i=1:n.numStatePoints-1], ((r.x[(i+1),1]-x0_)^2+(r.x[(i+1),2]-y0_)^2) <= (L_R + L_Rd)^2); # not constraining the first state
  newConstraint(r,LiDAR_con,:LiDAR_con);
  # ensure that the vehicle is at the edge of the LiDAR range at Tp
  #LiDAR_conF=@NLconstraint(mdl, (L_R - L_Rd)^2 <= ((r.x[n.numStatePoints,1]-x0_)^2+(r.x[n.numStatePoints,2]-y0_)^2) );
  #optimize(mdl,n,r)
  #dfs=Vector{DataFrame}(2);label_string = ["opt.","RK4"];
  #dfs[1]=x_to_DF(;t=(r.t_st),x=r.X[:,1],y=r.X[:,2],v=r.X[:,3],r=r.X[:,4],psi=r.X[:,5],sa=r.X[:,6],ux=r.X[:,7],ax=r.X[:,8],sr=r.U[:,1],jx=r.U[:,2]);
  #all_plots(dfs,["opt. inf"],results_dir,obs_data,s_data,pa)
  return mdl
end

m=nlp1()
tic()
status = solve(m)
tm = toq()
#=
DegeneracyHunter.printInfeasibleEquations(m, 0.1)
DegeneracyHunter.printInfeasibleEquations(mdl, 0.1)

DegeneracyHunter.printInactiveEquations(m)

ps = DegeneracyHunter.assembleProblemStats(m,status,tm)
DegeneracyHunter.printProblemStats(ps)
=#

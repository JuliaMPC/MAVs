using NLOptControl, JuMP, Parameters, VehicleModels
include("plots.jl");include("autonomousControl.jl");

pa=Vpara(u0_= 15.); @unpack_Vpara pa # other vehicles parameters

c=defineCase("c3");

# initialize
n1 = NLOpt();s1=Settings(;MPC=false,save=true,simulate=false,format=:png);

# define
pa1 = VparaKB();  @unpack_VparaKB pa1 # vehicle parameters  TODO do I need these?
X0 = [x0_,y0_,psi0_,u0_]; #TODO pull these states in
XF = [NaN,NaN,NaN,NaN];

#TODO when running in parallel run several with various constraints on x_min
XL = [x_min,y_min,psi_min,u_min];
XU = [x_max,y_max,psi_max,u_max];
CL = [sa_min,ax_min];
CU = [sa_max,ax_max];
n1 = define(n1,stateEquations=KinematicBicycle,numStates=4,numControls=2,X0=X0,XF=XF,XL=XL,XU=XU,CL=CL,CU=CU)

# build
n1 = configure(n1,Ni=2,Nck=[15,10];(:integrationMethod => :ps),(:integrationScheme => :lgrExplicit),(:finalTimeDV => true))
defineSolver(n1,solver=:IPOPT)
mdl1 = build(n1);

# addtional information
names = [:x,:y,:psi,:ux];
descriptions = ["X (m)","Y (m)","Yaw Angle (rad)","Longitudinal Velocity (m/s)"];
stateNames(n1,names,descriptions)
names = [:sr,:jx];
descriptions = ["Steering Angle (rad)","Longitudinal Acceleration (m/s^2)"];
controlNames(n1,names,descriptions);

#mXL=Any[false,false,false,false];mXU=Any[false,false,false,-1];  # set to false if you don't want to taper that side
#linearStateTolerances(n;mXL=mXL,mXU=mXU);

# setup OCP
params1 = [pa1];   # vehicle parameters
n1,r1 = OCPdef(mdl1,n1,s1,params1);
x_ref = 200; y_ref = 100; # define target TODO get from other model
@NLobjective(mdl1, Min,  n1.tf + (r1.x[end,1]-x_ref)^2 + (r1.x[end,2]-y_ref)^2);

# obstacles
Q = size(c.A)[1]; # number of obstacles
@NLparameter(mdl1, a[i=1:Q] == c.A[i]);
@NLparameter(mdl1, b[i=1:Q] == c.B[i]);
@NLparameter(mdl1, X_0[i=1:Q] == c.X0_obs[i]);
@NLparameter(mdl1, Y_0[i=1:Q] == c.Y0_obs[i]);
@NLparameter(mdl1, speed_x[i=1:Q] == c.s_x[i]);
@NLparameter(mdl1, speed_y[i=1:Q] == c.s_y[i]);

# obstacle postion after the intial postion
X_obs=@NLexpression(mdl1, [j=1:Q,i=1:n1.numStatePoints], X_0[j] + speed_x[j]*n1.tV[i]);
Y_obs=@NLexpression(mdl1, [j=1:Q,i=1:n1.numStatePoints], Y_0[j] + speed_y[j]*n1.tV[i]);

# constraint on position
obs_con=@NLconstraint(mdl1, [j=1:Q,i=1:n1.numStatePoints-1], 1 <= ((r1.x[(i+1),1]-X_obs[j,i])^2)/((a[j]+sm)^2) + ((r1.x[(i+1),2]-Y_obs[j,i])^2)/((b[j]+sm)^2));
newConstraint(r1,obs_con,:obs_con);

# solve
optimize(mdl1,n1,r1,s1)

# post process
r1.results_dir = string(main_dir,"/results_1","/test_1_",n1.solver,"/")
resultsDir(r1.results_dir);
using PrettyPlots, Plots
pyplot();
allPlots(n1,r1,s1,1)

idx=1;
pp=statePlot(n1,r1,s1,idx,1,2;(:lims=>false));
pp=obstaclePlot(n1,r1,s1,c,idx,pp;(:append=>true)); # add obstacles
pp=vehiclePlot(n1,r1,s1,c,idx,pp;(:append=>true)); # add the vehicle

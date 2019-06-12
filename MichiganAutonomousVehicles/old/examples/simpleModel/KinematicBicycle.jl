using NLOptControl, JuMP, Parameters, VehicleModels
include("plots.jl");include("autonomousControl.jl");
c=defineCase("c1");

# initialize
n1 = NLOpt();s1=Settings(;MPC=false,save=true,simulate=false,format=:png);

# define
pa1 = VparaKB();  @unpack_VparaKB pa1 # vehicle parameters
X0 = [x0_,y0_,psi0_,u0_]; #TODO pull these states in
XF = [NaN,NaN,NaN,NaN];
XL = [x_min,y_min,psi_min,u_min];
XU = [x_max,y_max,psi_max,u_max];
CL = [sa_min,ax_min];
CU = [sa_max,ax_max];
n1 = define(n1,stateEquations=KinematicBicycle,numStates=4,numControls=2,X0=X0,XF=XF,XL=XL,XU=XU,CL=CL,CU=CU)

# build
n1 = configure(n1,Ni=2,Nck=[15,10];(:integrationMethod => :ps),(:integrationScheme => :lgrExplicit),(:finalTimeDV => false),(:tf => 4.0))
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
@NLobjective(mdl1, Min, (r1.x[end,1]-x_ref)^2 + (r1.x[end,2]-y_ref)^2);

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

using NLOptControl
using VehicleModels
using Parameters
using DataFrames
using MAVs

# initializatin data
c=defineCase(;(:mode=>:caseStudy));
setMisc!(c;activeSafety=true,followPath=false,followDriver=false,PredictX0=false,FixedTp=false,tp=15.0,tex=0.3,max_cpu_time=0.26,Ni=3,Nck=[10,8,6]);
c.o.X0=[0.0,20.0,0.0];
c.o.Y0=[20.,50.0,10.0];
mdl,n,r,params,x,d=initializeSharedControl!(c)
global pa=params[1];

#=
s=Settings(;format=:png,MPC=false,reset=true,save=true);
X0 = [0.0, 0.0, 0.0, 0.0, pi/2]; SA=0.0; UX=15.0;
t_opt, sa_opt, t_sample, sa_sample = sharedControl(mdl,n,r,s,params,X0,SA,UX;Iter=22)

# test 1
X0 = [0.0, 0.0, 0.0, 0.0, pi/2]; SA=0.3; UX=15.0;
pass, AL, AU = feasible(mdl,d,n,r,s,params,X0,SA,UX)

# postProcess
using Plots, PrettyPlots
pyplot()
include("plots.jl");
r.results_dir = string(r.main_dir,"/results/","test_1/");
resultsDir(r.results_dir);
allPlots(n,r,s,1); # TODO should it be 1 or 2?
=#

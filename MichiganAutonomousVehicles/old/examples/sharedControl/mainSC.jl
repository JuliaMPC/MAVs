using NLOptControl
using VehicleModels
using Parameters
using DataFrames
using MAVs

try
  close(x.s1)
  close(x.s2)
end
mdl=0;n=0;r=0;params=0;c=0;x=0;d=0;pa=0;s=0;

# THINGS TO THINK ABOUT TODO
# 6) intialize obstacle field in intial solve
# 7) get the vehicle states from MATLAB so that I can plot things as well

c=defineCase(;(:mode=>:caseStudy));

# long prediction horizon
#setMisc!(c;activeSafety=false,followPath=true,followDriver=false,PredictX0=false,FixedTp=false,tp=8.0,tex=0.3,max_cpu_time=0.25,Ni=4,Nck=[12,10,8,6]);

#4.25 working
#setMisc!(c;activeSafety=false,followPath=true,followDriver=false,PredictX0=false,FixedTp=false,tp=5.0,tex=0.3,max_cpu_time=0.25,Ni=2,Nck=[10,10]);
#setWeights!(c;sr=0.05,path=10.0,driver=0.0)


setMisc!(c;activeSafety=false,followPath=true,followDriver=false,PredictX0=false,FixedTp=false,tp=5.0,tex=0.3,max_cpu_time=0.25,Ni=2,Nck=[10,10]);
setWeights!(c;sr=0.05,path=10.0,driver=0.0)



#4.21 -working
#setMisc!(c;activeSafety=false,followPath=true,followDriver=false,PredictX0=false,FixedTp=false,tp=5.0,tex=0.3,max_cpu_time=0.25,Ni=2,Nck=[10,10]);
#setWeights!(c;sr=0.08,path=10.0,driver=0.0)


mdl,n,r,params,x,d=initializeSharedControl!(c)

# setup UDP port and UDP communication
x.s1 = UDPSocket();bind(x.s1,ip"73.113.149.41",36880); # change this ip to the server (or julia code)
x.s2 = UDPSocket();bind(x.s2,ip"73.113.149.41",12000); # change this ip to the server (or julia code)

global pa=params[1];
global s=Settings(;reset=false,save=true,simulate=true,MPC=true);
r.eval_num=1;count=1;
# set infeasible_counter for active safety
x.infeasible_counter=100; x.infeasible_counter_max=3;  # initially the condition will be false
println("Getting Vehicle Position From MATLAB");
while(Bool(x.runJulia))
  if count!=r.eval_num
    println("Running model for the: ",r.eval_num," time");
  end
  count=r.eval_num;

  getPlantData!(n,params,x,c,r);

  updateX0!(n,r,x.X0;(:userUpdate=>true));

  if c.m.activeSafety
    checkFeasibility!(mdl,d,n,c,x,params[1],r;feas_tol=0.005);
    if !x.feasible
      x.infeasible_counter=1;
    else
      x.infeasible_counter+=1;
    end
  end

  if !c.m.activeSafety || !x.feasible || (x.infeasible_counter < x.infeasible_counter_max)
    println("Running Optimization");
    sharedControl!(mdl,n,r,s,params,x);   # rerun optimization
  end
  sendOptData!(n,r,x)
end
close(x.s1)
close(x.s2)

#include("postProcess.jl")

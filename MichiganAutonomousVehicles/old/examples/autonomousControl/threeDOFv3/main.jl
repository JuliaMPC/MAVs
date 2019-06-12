using MichiganAutonomousVehicles
using NLOptControl
using VehicleModels
using Parameters
using DataFrames
using PrettyPlots, Plots

case_name = "s1"
c = load(open(string(Pkg.dir("MichiganAutonomousVehicles"),"/config/planner/","RTPP",".yaml")))
c["vehicle"] = load(open(string(Pkg.dir("MichiganAutonomousVehicles"),"/config/vehicle/","hmmwv",".yaml")))
c["goal"] = load(open(string(Pkg.dir("MichiganAutonomousVehicles"),"/config/case/",case_name,".yaml")))["goal"]
c["X0"] = load(open(string(Pkg.dir("MichiganAutonomousVehicles"),"/config/case/",case_name,".yaml")))["X0"]
c["obstacle"] = load(open(string(Pkg.dir("MichiganAutonomousVehicles"),"/config/case/",case_name,".yaml")))["obstacle"]
setConfig(c,"misc";(:N=>15),(:model=>:ThreeDOFv3),(:solver=>:KNITRO),(:integrationScheme=>:trapezoidal))
setConfig(c,"X0";(:ux=>5.0))# the type needs to be a float
# otherwise, this will pop up: WARNING: Ipopt finished with status Invalid_Number_Detected
fixYAML(c)   # fix messed up data types
n = initializeAutonomousControl(c);
simMPC!(n;updateFunction=updateAutoParams!,checkFunction=checkCrash)

function mpcSim(n,idx)
  mpcPlot(n,idx)
end
# (:mpc_lines=>[(6.5,:blue,:solid)])
# (:mpc_markers =>(:circle,:blueviolet,0.0,0.0)),
a = 2
plotSettings(;(:X0p=>true), (:mpc_markers =>(:circle,:blueviolet,0.0,1.)),(:polyPts=>true),(:simulate=>true),(:plant=>true),(:plantOnly=>false),(:size=>(a*900,a*600)),(:format=>"png"));
anim = @animate for idx in 1:5#length(n.r.ocp.dfs)
  mpcSim(n,idx)
end
cd(n.r.resultsDir)
  gif(anim,"mainSim.gif",fps=Int(ceil(1/n.mpc.v.tex)));
  run(`ffmpeg -f gif -i mainSim.gif RESULT.mp4`)
cd(n.r.mainDir)

savePlantData!(n)
saveOptData(n)

using BenchmarkTools
using MAVs, NLOptControl, VehicleModels

# change defaults
BenchmarkTools.DEFAULT_PARAMETERS.seconds = 2.50
BenchmarkTools.DEFAULT_PARAMETERS.samples = 1

const SUITE = BenchmarkGroup()

#case_names = ["s1","s2","s3","s4","s5","s6"]
SUITE["test"] = BenchmarkGroup(["case","solver"])
planner_name = "RTPP"
vehicle_name = "hmmwv"
models =[:ThreeDOFv2, :KinematicBicycle2]
model = models[1]

for case_name in ("s1","s2")
    for solver in (:KNITRO,:Ipopt)
        c = load(open(string(Pkg.dir("MAVs"),"/config/planner/",planner_name,".yaml")))
        c["vehicle"] = load(open(string(Pkg.dir("MAVs"),"/config/vehicle/",vehicle_name,".yaml")))
        c["goal"] = load(open(string(Pkg.dir("MAVs"),"/config/case/",case_name,".yaml")))["goal"]
        c["X0"] = load(open(string(Pkg.dir("MAVs"),"/config/case/",case_name,".yaml")))["X0"]
        c["obstacle"] = load(open(string(Pkg.dir("MAVs"),"/config/case/",case_name,".yaml")))["obstacle"]
        setConfig(c, "misc";(:N=>15),(:model=>model),(:solver=>solver), (:integrationScheme=>:trapezoidal))
        fixYAML(c)   # fix messed up data types
        n = initializeAutonomousControl(c);
        SUITE["test"][case_name,string(solver)] = @benchmarkable simMPC!($n;updateFunction=updateAutoParams!,checkFunction=checkCrash)
    end
end

run(SUITE)

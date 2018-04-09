using RobotOS
@rosimport geometry_msgs.msg: Point, Pose, Pose2D, PoseStamped, Vector3, Twist
@rosimport nloptcontrol_planner.msg: Control
@rosimport nav_msgs.msg: Path

rostypegen()
using geometry_msgs.msg
using nloptcontrol_planner.msg
using nav_msgs.msg

import YAML

using VehicleModels
using NLOptControl
using MAVs
using PyCall

@pyimport tf.transformations as tf

println("initializing nloptcontrol_planner node ...")
init_node("nloptcontrol_planner")

# message for solution to optimal control problem
plannerNamespace = RobotOS.get_param("system/nloptcontrol_planner/namespace")
pub = Publisher{Control}(string(plannerNamespace,"/control"), queue_size=10)
pub_path = Publisher{Path}("/path", queue_size=10)

sub = Subscriber{Control}(string(plannerNamespace, "/control"), setTrajParams, queue_size = 10)

# using the filenames set as rosparams, the datatypes of the parameters get messed up if they are put on the ROS server
# and then loaded into julia through RobotOS.jl; but less is messed up by loading using YAML.jl
case = YAML.load(open(RobotOS.get_param("case_params_path")))["case"]
planner = YAML.load(open(RobotOS.get_param("planner_params_path")))["planner"]["nloptcontrol_planner"]
vehicle = YAML.load(open(RobotOS.get_param("vehicle_params_path")))["vehicle"]["nloptcontrol_planner"]

c = YAML.load(open(string(Pkg.dir("MAVs"),"/config/empty.yaml")))
c["vehicle"] = convert(Dict{Symbol,Any}, vehicle)  # need symbols to work with VehicleModels.jl
c["weights"] = planner["weights"]
c["misc"] = planner["misc"]
c["solver"] = planner["solver"]
c["tolerances"] = planner["tolerances"]
c["X0"] = case["actual"]["X0"]
c["goal"] = case["goal"]

if RobotOS.get_param("system/nloptcontrol_planner/flags/known_environment")
  c["obstacle"] = case["actual"]["obstacle"]
else  # NOTE currently the the python parcer does not like assumed/obstacle format!, this will fail!
  c["obstacle"] = case["assumed"]["obstacle"]
end
# fix messed up data types
for keyA in ["weights", "misc", "X0", "solver", "tolerances"]
  for (key,value) in c[keyA]
    if isequal(value, "NaN"); c[keyA][key] = NaN; end
  end
end
for keyA in ["misc"]
  for (key,value) in c[keyA]
    if isequal(typeof(c[keyA][key]),String); c[keyA][key] = Symbol(c[keyA][key]); end
  end
end

#########
if c["misc"]["model"]==:ThreeDOFv2
 n = initializeThreeDOFv2(c);
elseif c["misc"]["model"]==:KinematicBicycle
  n = initializeKinematicBicycle(c);
else
  error(string("c[misc][model] = " ,c["misc"]["model"]," needs to be set to either; :ThreeDOFv2 || :KinematicBicycle "))
end

setInitStateParams(c)

if RobotOS.get_param("system/nloptcontrol_planner/flags/known_environment")
  setInitObstacleParams(c)
end




########

planner_name = "RTPP"
cases = ["s1","s2","s3"]
vehicle_name = "hmmwv"
a = load(open(string(Pkg.dir("MAVs"),"/config/empty.yaml")))
a["vehicle"] = Vpara()

for fn in fieldnames(a["vehicle"])
  println(typeof(fn))
      if !isequal(c["vehicle"][fn],a["vehicle"][fn])
        println(fn)
      end
end

using RobotOS
@rosimport std_msgs.msg: Float64MultiArray
@rosimport geometry_msgs.msg: Point
rostypegen()
using std_msgs.msg       # subscribe to topic
using geometry_msgs.msg


#TODO: use proper data type(s) here
#       to receive message from ROS (and actually from Gazebo)
function callback(msg::Float64MultiArray)
   println(msg)
end
# https://github.com/jdlangs/RobotOS.jl

# TODO: here is an example of pushing data in fixed rate
#       (just like there is a timer)
function loop(pub_obj)
   loop_rate = Rate(1)
   npt = Float64MultiArray()
   npt.data = Array{Float64}(6)
   npt.data[6] = 0.0
   while ! is_shutdown()
     npt.data[1] += 0.2
     npt.data[2] = 0.0
     npt.data[3] = 0.0
     npt.data[4] = 0.0
     npt.data[5] = 0.0
     npt.data[6] += 0.2  # NOTE this might need to be set to 0

     # send information to ROS (then Gazebo)
     publish(pub_obj, npt)
     rossleep(loop_rate)
   end
end

function main()
   #init_node("rosjl_example") # what is this?

   pub = Publisher{Float64MultiArray}("gazebo_client/test", queue_size=10)
   sub = Subscriber{Float64MultiArray}("gazebo_client/sendscan", callback, (), queue_size=10)

   loop(pub)
end

if ! isinteractive()
   main()
end








# set up roscore
#@pyimport roslaunch

# setup for the Python script
#using PyCall
#@pyimport imp
#Node=imp.load_source("Node","pythonNode.py");

include("utils.jl") # functions

function __init__()
  addprocs(3);

  # start up a roscore ?? or is it already running
  #roslaunch.roscore

  # run the ptthon script Echo.py to start ROS node to cummunicate between julia and Gazebo
  node=remotecall(Node[:main],2,());
  println("ROS node Loaded!");

  # run the julia script handler.jl to get point cloud information from Gazebo
  data=remotecall(main,3,());
  println("Ready to Collect Data From Gazebo!");

  # start Gazebo
  gazebo=remotecall(run,4,(`gazebo velodyne.world`));
  println("Gazebo LiDAR model running!");

end

export data

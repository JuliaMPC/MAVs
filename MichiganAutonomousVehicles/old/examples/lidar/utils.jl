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

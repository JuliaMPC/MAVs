#!/usr/bin/env julia
using RobotOS

"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/15/2018, Last Modified: 3/15/2018 \n
--------------------------------------------------------------------------------------\n
"""
function main()
  println("starting bootstrap node ...")
  init_node("bootstrap_system")

  if RobotOS.get_param("system/vehicle_description/flags/running")
    while(!RobotOS.get_param("system/vehicle_description/flags/lidar_initialized"))
      println("waiting on move_vehicle.jl in vehicle_description ...")
      sleep(2)
    end

    while(!RobotOS.get_param("system/vehicle_description/flags/obstacles_initialized"))
      println("waiting on move_obstacles.jl in vehicle_description ...")
      sleep(2)
    end
  end

  if RobotOS.get_param("system/nloptcontrol_planner/flags/running")
    while(!RobotOS.get_param("system/nloptcontrol_planner/flags/initilized"))
      println("waiting on obstacle_avoidance.jl in nloptcontrol_planner ...")
      sleep(5)
    end
  end

  println("system has been initialized!")
  RobotOS.set_param("system/paused",false)
end

if !isinteractive()
    main()
end

#!/usr/bin/env julia
using RobotOS

"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/15/2018, Last Modified: 4/6/2018 \n
--------------------------------------------------------------------------------------\n
"""
function main()
  println("starting bootstrap node ...")
  init_node("bootstrap_system")

  if RobotOS.has_param("system/vehicle_description/flags/running")
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
  end

  if RobotOS.has_param("system/obstacle_detector/flags/running")
    if RobotOS.get_param("system/obstacle_detector/flags/running")
      while(!RobotOS.get_param("system/obstacle_detector/flags/initialized"))
        println("waiting on obstacle_detector to be initialized ...")
        sleep(2)
      end
    end
  end

  if RobotOS.has_param("system/ros_base_planner/flags/running")
    if RobotOS.get_param("system/ros_base_planner/flags/running")
      while(!RobotOS.get_param("system/ros_base_planner/flags/initialized"))
        println("waiting on ros_base_planner ...")
        sleep(2)
      end
    end
  end

  if RobotOS.has_param("system/chrono/flags/running")
    if RobotOS.get_param("system/chrono/flags/running")
      while(!RobotOS.get_param("system/chrono/flags/initialized"))
        println("waiting on chrono ...")
        sleep(2)
      end
    end
  end

  if RobotOS.has_param("system/nloptcontrol_planner/flags/running")
    if RobotOS.get_param("system/nloptcontrol_planner/flags/running")
      while(!RobotOS.get_param("system/nloptcontrol_planner/flags/initialized"))
        println("waiting on obstacle_avoidance.jl in nloptcontrol_planner ...")
        sleep(5)
      end
    end
  end

  if RobotOS.has_param("system/shutdown/flags/running")
    if RobotOS.get_param("system/shutdown/flags/running")
      while(!RobotOS.get_param("system/shutdown/flags/initialized"))
        println("waiting on shudown node to initialize in system ...")
        sleep(5)
      end
    end
  end

  println("system has been initialized!")
  RobotOS.set_param("system/flags/paused",false)
  RobotOS.set_param("system/flags/initialized",true)
end

if !isinteractive()
    main()
end

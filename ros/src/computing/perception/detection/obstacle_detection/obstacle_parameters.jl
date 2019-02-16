#!/usr/bin/env julia
using RobotOS
@rosimport obstacle_detector.msg: Obstacles, CircleObstacle
rostypegen()
using obstacle_detector.msg

function setObstacleParams(msg::Obstacles)
  L = length(msg.circles)
  if L > 0
    r=zeros(L);x=r;y=r;vx=r;vy=r;
    r = (); x = (); y = (); vx = (); vy = ();
    for i in 1:L
      r = (r..., msg.circles[i].radius)
      x = (x..., msg.circles[i].center.x)
      y = (y..., msg.circles[i].center.y)
      vx = (vx..., msg.circles[i].velocity.x)
      vy = (vy..., msg.circles[i].velocity.y)
    end
    @show vy

    # update obstacle field parameters
    RobotOS.set_param("obstacle/radius",r)
    RobotOS.set_param("obstacle/x",x)
    RobotOS.set_param("obstacle/y",y)
    RobotOS.set_param("obstacle/vx",vx)
    RobotOS.set_param("obstacle/vy",vy)
  else
    RobotOS.set_param("obstacle/radius",NaN)
    RobotOS.set_param("obstacle/x",NaN)
    RobotOS.set_param("obstacle/y",NaN)
    RobotOS.set_param("obstacle/vx",NaN)
    RobotOS.set_param("obstacle/vy",NaN)
  end
  return nothing
end

init_node("obstacle_params")
sub = Subscriber{Obstacles}("/obstacles", setObstacleParams, queue_size = 10)
spin()

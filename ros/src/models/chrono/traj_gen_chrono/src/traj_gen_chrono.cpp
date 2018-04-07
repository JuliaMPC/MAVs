#include "ros/ros.h"
#include <ros/console.h>
#include "std_msgs/String.h"
#include <sstream>
#include <vector>
#include "nloptcontrol_planner/Control.h"
//#include "traj_gen_chrono/Control.h"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */


  ros::init(argc, argv, "Reference");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle a;
  std::vector<double> x1;
  std::vector<double> y1;
  std::vector<double> x2(2,200);
  std::vector<double> y2(2,0);

  ros::Publisher pub = a.advertise<nloptcontrol_planner::Control>("/nloptcontrol/control", 10);
  nloptcontrol_planner::Control control_info;
  int control_num = 10;
  std::vector<double> control_t(control_num,0.0);
  std::vector<double> control_sa(control_num,0.0);
  std::vector<double> control_vx(control_num,0.0);
  control_info.x = std::vector<double>(control_num,0.0);
  control_info.y = std::vector<double>(control_num,0.0);
  control_info.psi = std::vector<double>(control_num,0.0);
  std::string planner_namespace;
  a.getParam("system/planner",planner_namespace);
//  XmlRpc::XmlRpcValue x1,y1;
  // a.getParam("vehicle/chrono/nloptcontrol_planner/traj/x",x1);
  // a.getParam("vehicle/chrono/nloptcontrol_planner/traj/yVal",y1);

    ros::WallRate loop_rate(2);

    double count = 0;
    while (ros::ok())
    {
      double secs = ros::Time::now().toSec();
      for(int i = 0; i < control_num; i++){
        control_t[i] = secs + i - 1;
        control_sa[i] = i * 0.01 + count * 0.01 / 2.0;
        control_vx[i] = i * 0.5 + count * 0.5 / 2.0;
        std::cout << "The reference steering angle: " << control_sa[1] << std::endl;
      }
      control_info.t = control_t;
      control_info.sa = control_sa;
      control_info.vx = control_vx;

      /* path....
      x2[0]= 200;
      x2[1]= 200 + count;
      y2[0] = 0;
      y2[1] = 50 ;
      a.setParam("vehicle/chrono/default/traj/x",x2);
      a.setParam("vehicle/chrono/default/traj/yVal",y2);
      */

      ros::spinOnce();
      pub.publish(control_info);
      loop_rate.sleep();
      count ++;
    }


  return 0;
}

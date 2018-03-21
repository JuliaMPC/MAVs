#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"

//This cpp file publishes simulation time subscribed from Chrono.

rosgraph_msgs::Clock time(0);

int main(int argc, char **argv)
{
// create node sim_time_publisher
  ros::init(argc, argv, "sim_time_publisher");


  ros::NodeHandle n;

// publish time
  ros::Publisher time_pub = n.advertise<rosgraph_msgs::Clock>("/clock", 1000);

  ros::Rate loop_rate(1000);

  while (ros::ok()){
    time_pub.publish(time);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

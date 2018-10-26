#include "ros/ros.h"
#include <ros/console.h>
#include "std_msgs/String.h"
#include <sstream>
#include "traj_gen_chrono/Control.h"

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
//  XmlRpc::XmlRpcValue x1,y1;
  a.getParam("/x_traj",x1);
  a.getParam("/y_traj",y1);
/*  for (int32_t i = 0; i < x1.size(); ++i)
  {
    ROS_ASSERT(x1[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    ROS_ASSERT(y1[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
  } */
  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher traj_desired_pub =  a.advertise<traj_gen_chrono::Control>("desired_ref", 1);

  ros::Rate loop_rate(1);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {


    traj_gen_chrono::Control data_out;
/*
    std::vector<double> x(2);
    std::vector<double> y(2);
    x[0]=200;
    x[1]=200;
    y[0]=0;
    y[1]=50;
    */
    data_out.x=x1;
    data_out.y=y1;
    traj_desired_pub.publish(data_out);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}

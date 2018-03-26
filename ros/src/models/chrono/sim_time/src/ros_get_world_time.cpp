#include <math.h>

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo_client.hh>

#include <ros/ros.h>
#include <ros/subscribe_options.h>

int main(int argc, char** argv)
{
  gazebo::client::setup(argc, argv);
  ros::init(argc, argv, "get_world_time_test");

  // Gazebo WorldControl Topic
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
  // Initialize the ROSNode
  ros::NodeHandle* rosnode = new ros::NodeHandle();

  gazebo::transport::PublisherPtr pub = node->Advertise<gazebo::msgs::WorldControl>("~/world_control");

  // Waits for simulation time update.
  ros::Time last_ros_time_;
  bool wait = true;
  int step_simulation_initial;
  rosnode->getParam("system/step_simulation", step_simulation_initial);
  while (wait)
  {
    last_ros_time_ = ros::Time::now();
    // std::cout << "\t\t\t Attempted getting sim time: " << last_ros_time_ << std::endl;

    // if (last_ros_time_.toSec() > 0)
      //wait = false;
      // std::cout << "Current Simulation Time: " << last_ros_time_ << std::endl;

    // Publish the step message for the simulation.
    gazebo::msgs::WorldControl msg;
    msg.set_step(1);
    pub->Publish(msg);

    // get simulation step
    rosnode->getParam("system/step_simulation", step_simulation);
    // Wait for 1 second and allow ROS to complete as well.
    gazebo::common::Time::NSleep(step_simulation*1000000);
    // std :: cout << "Step time in nano seconds: " << step_simulation << std::endl;
    ros::spinOnce();
  }

  gazebo::shutdown();

  return 0;
}

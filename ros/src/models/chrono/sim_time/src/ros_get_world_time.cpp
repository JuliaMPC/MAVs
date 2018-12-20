// http://answers.gazebosim.org/question/12138/perform-one-simulation-step/
// https://bitbucket.org/osrf/gazebo/src/5299916de5cce4a5f3baf8acf75cf6bff68f8184/gazebo/msgs/world_control.proto?at=default&fileviewer=file-view-default
// http://answers.gazebosim.org/question/18995/gazebo-simulated-time-control-step-by-step/
//https://github.com/jaredmoore/ROS-Gazebo-Examples/blob/master/gazebo_step_world/world_step.cc
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

  double simulation_time;
  rosnode->getParam("state/t", simulation_time);

  bool is_init;
  rosnode->setParam("system/sim_time/flags/initialized", true);
  rosnode->getParam("system/flags/initialized", is_init);
  while (!is_init) {
      rosnode->getParam("system/flags/initialized", is_init);
  }
  double ros_time;
  while (1)
  {
    // Publish the step message for the simulation.
    gazebo::msgs::WorldControl msg;

    // get current times
    rosnode->getParam("state/t", simulation_time);
    ros_time = ros::Time::now().toSec();

    // get Gazebo time and if it is less than Chrono time then step, else pause
    if(ros_time < simulation_time){
      msg.set_pause(0);
      msg.set_step(1);
      pub->Publish(msg);
    }
    else{
      msg.set_pause(1);
      pub->Publish(msg);
    }

  }
  gazebo::shutdown();
  return 0;
}

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

  // Waits for simulation time update.
//  ros::Time last_ros_time_;

//  double simulation_time_pre;
  double simulation_time;
  rosnode->getParam("state/t", simulation_time);
//  double delta_time = 0;

  bool is_init;
  rosnode->setParam("system/sim_time/flags/initialized", true);
  rosnode->getParam("system/flags/initialized", is_init);
  while (!is_init) {
      rosnode->getParam("system/flags/initialized", is_init);
  }
  double ros_time;
  while (1)
  {
    // last_ros_time_ = ros::Time::now();
    // std::cout << "\t\t\t Attempted getting sim time: " << last_ros_time_ << std::endl;

    // if (last_ros_time_.toSec() > 0)
      //wait = false;

    // Publish the step message for the simulation.
    gazebo::msgs::WorldControl msg;

    // get Gazebo time and if it is less than Chrono time then step, else pause
    // get simulation step
    rosnode->getParam("state/t", simulation_time);

    //delta_time = simulation_time - simulation_time_pre;
    ros_time = ros::Time::now().toSec();
    //if(delta_time)  pub->Publish(msg);
    //pub->Publish(msg);
    if(ros_time < simulation_time){
      msg.set_pause(0);
      msg.set_step(1);
      pub->Publish(msg);
      ROS_INFO("STEP");
    }
    else{
      msg.set_pause(1);
      pub->Publish(msg);
      ROS_INFO("PAUSE");
    }
    std :: cout << "chrono: " << simulation_time << " , ros: " << ros_time << std::endl;

    // Wait for 1 msecond and allow ROS to complete as well.

  //  gazebo::common::Time::MSleep(delta_time*100.0);
  //  simulation_time_pre = simulation_time;
    // std :: cout << "Time in seconds: " << simulation_time << " Delta_time: " << delta_time << std::endl;
    // ros::spinOnce();
  }

  gazebo::shutdown();

  return 0;
}

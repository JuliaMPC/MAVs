#include <sdf/sdf.hh>
#include <ignition/math/Pose3.hh>
#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

#include <ros/ros.h>
#include <ros/subscribe_options.h>

/// \example examples/plugins/world_edit.cc
/// This example creates a WorldPlugin, initializes the Transport system by
/// creating a new Node, and publishes messages to alter gravity.
namespace gazebo
{
  class WorldEdit : public WorldPlugin
  {
    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
      double step_size = 0.001;
      ros::NodeHandle* rosnode = new ros::NodeHandle();
      if(ros::param::has("system/params/step_size")){
        ROS_INFO("++++++++++++++++++get step_size!++++++++++++++++");
        rosnode->getParam("system/params/step_size", step_size);
      }

      // Create a new transport node
      transport::NodePtr node(new transport::Node());

      // Initialize the node with the world name
      node->Init(_parent->Name());

      // Create a publisher on the ~/physics topic
      transport::PublisherPtr physicsPub =
        node->Advertise<msgs::Physics>("~/physics");

      msgs::Physics physicsMsg;
      physicsMsg.set_type(msgs::Physics::ODE);

      // Set the step time
      physicsMsg.set_max_step_size(step_size);
      physicsMsg.set_real_time_update_rate(1.0/step_size);

      physicsPub->Publish(physicsMsg);
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(WorldEdit)
}

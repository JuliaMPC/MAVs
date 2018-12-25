#include <signal.h>
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <std_msgs/Empty.h>

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;
bool is_shutdown_hook_initiated = false;
std::vector<std::string> shutdown_initiation_flags;
std::vector<std::string> shutdown_completion_flags;

// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    g_request_shutdown = 1; // Set flag
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}

void shutdownSystem() {
  ros::shutdown();
}

bool areAllShutdownCompletionFlagsTrue() {
  bool val;
  for (int i = 0; i < shutdown_completion_flags.size(); i++) {
    if (ros::param::has(shutdown_completion_flags[i])) {
      ros::param::get(shutdown_completion_flags[i], val);
      if (!val) {
        ROS_INFO("Waiting for %s flag to be true\n", shutdown_completion_flags[i].c_str());
        return false;
      }
    } else {
      ROS_WARN("Following parameter is expected for shutdown completion but not found: %s.\n", shutdown_completion_flags[i].c_str());
    }
  }
  return true;
}

bool isShutDownAllowed() {
  bool override_shutdown_hook = false;
  if (ros::param::has("system/flags/override_shutdown_hook")) {
    ros::param::get("system/flags/override_shutdown_hook", override_shutdown_hook);
    if (override_shutdown_hook) {
      return true;
    } else {
      return areAllShutdownCompletionFlagsTrue();
    }
  } else {
    return areAllShutdownCompletionFlagsTrue();
  }
}

void checkForShutdownRequest() {
  for (int i = 0; i < shutdown_initiation_flags.size(); i++) {
    if (ros::param::has(shutdown_initiation_flags[i])) {
      bool initiate_shutdown;
      ros::param::get(shutdown_initiation_flags[i], initiate_shutdown);
      if (initiate_shutdown) {
        ros::param::set("system/flags/shutdown", true);
        ROS_INFO("Shutdown initiation flag triggered!!!! = %s\n", shutdown_initiation_flags[i].c_str());
        g_request_shutdown = 1;
        break;
      }
    } else {
      // for debug purposes
      // ROS_WARN("Following parameter is expected for shutdown initiation but not found: %s.\n", shutdown_initiation_flags[i].c_str());
    }
  }
}

int main(int argc, char** argv)
{
  // Override SIGINT handler
  ros::init(argc, argv, "system_shutdown", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;

  signal(SIGINT, mySigIntHandler);
  bool shutdown_process_started = false;
  std_msgs::Empty shutdown_msg;
  // Override XMLRPC shutdown
  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

  ros::Publisher shutdown_requested = nh.advertise<std_msgs::Empty>("shutdown_requested", 1000);

  ros::param::get("system/shutdown/params/shutdown_initiation_flags",shutdown_initiation_flags);
  ros::param::get("system/shutdown/params/shutdown_completion_flags",shutdown_completion_flags);

  ros::param::set("system/shutdown/flags/initialized", true);
  // Do our own spin loop
  ros::Rate loop_rate(1);
  while(ros::ok()) {
    if (g_request_shutdown) {
      if (!shutdown_process_started) {
        ROS_INFO("----System shutdown captured----\n");
        shutdown_requested.publish(shutdown_msg);
      }
      if (isShutDownAllowed()) {
        shutdownSystem();
      }
      shutdown_process_started = true;
    } else {
      checkForShutdownRequest();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  //ros::spin();
  return 0;
}

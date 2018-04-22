#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main (int argc, char **argv)
{
  ros::init(argc, argv, "move_base_goal_client");

  ros::NodeHandle n;

  float x_g;
  float y_g;
  float psi_g;
  std::string planner_ns = "";
  bool goal_known = false;
  ros::param::get("system/planner", planner_ns);

  ros::param::get("system/" + planner_ns + "/flags/goal_known", goal_known);

  // create the action client
  // true causes the client to spin its own thread
  MoveBaseClient ac("move_base", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");

  if(goal_known == 1) {
    ros::param::get("case/goal/x", x_g);
    ros::param::get("case/goal/yVal", y_g);
    ros::param::get("case/goal/psi", psi_g);

    //{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}
    geometry_msgs::PoseStamped goal = geometry_msgs::PoseStamped();
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x = x_g;
    goal.pose.position.y = y_g;
    goal.pose.orientation.z = psi_g;
    move_base_msgs::MoveBaseGoal action_goal;
    action_goal.target_pose = goal;

    std::cout << "Sending goal... " << "\n";
    ac.sendGoal(action_goal);
    std::cout << "Goal sent!" << "\n";
    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
      ROS_INFO("Action did not finish before the time out.");

  } else {
    ROS_INFO("No goal to send.");
  }

  //exit
  return 0;
}

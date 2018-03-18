#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>

std::string planner_ns;

void poseCallback(const nav_msgs::Path path) {
  std::vector<float> x_p;
  std::vector<float> y_p;
   for (int i = 0; i < path.poses.size(); i++) {
     x_p.push_back(path.poses[i].pose.position.x);
     y_p.push_back(path.poses[i].pose.position.y);
   }

   ros::param::set("vehicle/chrono/" + planner_ns + "/traj/" + "x", x_p);
   ros::param::set("vehicle/chrono/" + planner_ns + "/traj/" + "y", y_p);
   ros::param::set("system/" + planner_ns + "/flags/initialized", true);
}


int main(int argc, char** argv) {

  ros::init(argc, argv, "navsPathConverter");

  ros::NodeHandle n;

  float x_g;
  float y_g;
  float psi_g;

  //Setting goal
  ros::param::get("system/planner", planner_ns);
  ros::param::get("case/goal/x", x_g);
  ros::param::get("case/goal/yVal", y_g);
  ros::param::get("case/goal/psi", psi_g);

  //Setting up current pose
  //ros::param::get("system/planner", planner_ns);

  ros::Subscriber sub = n.subscribe<nav_msgs::Path>("/move_base/TrajectoryPlannerROS/local_plan", 1000, poseCallback);

  ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1, true);

  //{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}
  geometry_msgs::PoseStamped goal = geometry_msgs::PoseStamped();
  goal.header.frame_id = "/map";
  goal.header.stamp = ros::Time::now();
  goal.pose.position.x = x_g;
  goal.pose.position.y = y_g;
  goal.pose.orientation.z = psi_g;
  goal_pub.publish(goal);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {

    ros::spinOnce();

    loop_rate.sleep();
  }

}

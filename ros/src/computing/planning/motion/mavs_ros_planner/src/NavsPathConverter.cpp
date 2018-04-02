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
   ros::param::set("vehicle/chrono/" + planner_ns + "/traj/" + "yVal", y_p);
   ros::param::set("system/" + planner_ns + "/flags/initialized", true);
}


int main(int argc, char** argv) {

  ros::init(argc, argv, "navsPathConverter");

  ros::NodeHandle n;

  ros::param::get("system/planner", planner_ns);

  ros::Subscriber sub = n.subscribe<nav_msgs::Path>("/move_base/NavfnROS/plan", 1000, poseCallback);

  ros::spin();
}

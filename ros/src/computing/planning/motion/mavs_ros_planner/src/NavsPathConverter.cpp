#include <ros/ros.h>

void poseCallback(const nav_msgs::Path& path, std::string ns) {
   for (int i = 0; i < path.poses.size(); i++) {
     ros::param::set("/" + ns + "/" + "x", path.poses[i].pose.position.x));
     ros::param::set("/" + ns + "/" + "y", path.poses[i].pose.position.y));
   }
   ros::param::set("system/" + ns + "/flags/initialized", true);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "navsPathConverter");

  ros::NodeHandle n;

  std::string planner_ns;
  ros::param::get("system/planner", planner_ns);

  //Setting up current pose
  ros::param::get("system/planner", planner_ns);

  ros::Subscriber sub = n.subscribe<nav_msgs::Path>("local_plan", 1000, boost::bind(poseCallback, _1, planner_ns));

  ros::spin();
}

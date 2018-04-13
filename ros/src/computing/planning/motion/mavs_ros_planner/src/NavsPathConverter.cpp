#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>

std::string planner_ns;
ros::NodeHandle * nh;

void poseCallback(const nav_msgs::Path path) {
  std::vector<double> x_p;
  std::vector<double> y_p;
   for (int i = 0; i < path.poses.size(); i++) {
     x_p.push_back(path.poses[i].pose.position.x);
     y_p.push_back(path.poses[i].pose.position.y);
   }

   if (x_p.size() >= 2 && y_p.size() >= 2) {


     std::vector<double> x_pt(1000, 5.0);
     std::vector<double> y_pt(1000, 5.0);
     //nh->setParam("/vehicle/chrono/" + planner_ns + "/traj/" + "x", x_p);
     //nh->setParam("/vehicle/chrono/" + planner_ns + "/traj/" + "yVal", y_p);
     //while (!ros::master::check()){}
     ros::param::set("vehicle/chrono/" + planner_ns + "/traj/" + "x1", x_pt);
     while(!nh->getParamCached("vehicle/chrono/" + planner_ns + "/traj/" + "x1", x_pt)){}
     std::cout<<"MASTER DATA2"<<"\n";
     ros::param::set("/vehicle/chrono/" + planner_ns + "/traj/" + "yVal", y_pt);
     while(!nh->getParamCached("vehicle/chrono/" + planner_ns + "/traj/" + "yVal", y_pt)){}
     std::cout << "AFTER PARAMETER SET"<<"\n";
     //nh->setParam("/system/" + planner_ns + "/flags/initialized", true);
     ros::param::set("/system/" + planner_ns + "/flags/initialized", true);
   }
}


int main(int argc, char** argv) {

  ros::init(argc, argv, "navsPathConverter");

  ros::NodeHandle n;
  nh = &n;

  ros::param::get("system/planner", planner_ns);

  ros::Subscriber sub = n.subscribe<nav_msgs::Path>("/move_base/NavfnROS/plan", 1000, poseCallback);
  //n.setParam("/system/" + planner_ns + "/flags/path_converter_initialized", true);
  //ros::spin();
  while (1)
  {
    ros::spinOnce();
  }
}

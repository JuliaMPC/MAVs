#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster1;
  tf::TransformBroadcaster broadcaster2;

  while(n.ok()){
    broadcaster1.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.5)),
        ros::Time::now(),"base_footprint", "base_link"));

    broadcaster2.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1.55, 0.0, 0.738)),
        ros::Time::now(),"base_link", "velodyne_top_link"));
    r.sleep();
  }
}

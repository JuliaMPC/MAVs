#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster1;
  tf::TransformBroadcaster broadcaster2;

  while(n.ok()){
    /*
    double yaw = 1.57079632679;
    double pitch = 0;
    double roll = 0;
    double cy = cos(yaw * 0.5); // 0.7071
    double sy = sin(yaw * 0.5); // 0.7071
    double cr = cos(roll * 0.5); // 1
    double sr = sin(roll * 0.5);  // 0
    double cp = cos(pitch * 0.5); // 1
    double sp = sin(pitch * 0.5); // 0

    // q4 is the normal
    double q4 = cy * cr * cp + sy * sr * sp;
    double q1 = cy * sr * cp - sy * cr * sp;
    double q2 = cy * cr * sp + sy * sr * cp;
    double q3 = sy * cr * cp - cy * sr * sp;
*/

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

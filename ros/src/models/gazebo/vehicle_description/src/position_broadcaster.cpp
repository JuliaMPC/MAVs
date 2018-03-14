#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>

void poseCallback(const geometry_msgs::Pose& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg.position.x,msg.position.y,msg.position.z) );
  tf::Quaternion q;
  q.setRPY(msg.orientation.x,msg.orientation.y,msg.orientation.z);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"map", "base_footprint")); //TODO get this parameter
}

int main(int argc, char** argv){
  //ros::Rate rate(100.0);
  ros::init(argc, argv, "tf_position_broadcaster");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/hmmwv/base_footprint_link_pose", 10, &poseCallback);

  ros::spin();
  return 0;
};

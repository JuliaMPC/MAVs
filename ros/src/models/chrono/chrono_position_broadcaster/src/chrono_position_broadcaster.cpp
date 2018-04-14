#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
/*
void poseCallback(const geometry_msgs::Pose& msg){

}
*/
int main(int argc, char** argv){
  //ros::Rate rate(100.0);
  ros::init(argc, argv, "tf_position_broadcaster");

  ros::NodeHandle node;
  double x,y,z,r,p,yaw;


  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  while(1){
    node.getParam("state/chrono/x", x) ;
    node.getParam("state/chrono/yVal",y);
    node.getParam("state/chrono/X0/z",z);
    node.getParam("state/chrono/psi",yaw); //in radians
    node.getParam("state/chrono/theta",p); //in radians
    node.getParam("state/chrono/phi",r); //in radians
    transform.setOrigin( tf::Vector3(x,y,z) );
    q.setRPY(r,p,y);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"map", "base_link"));
  }
  return 0;
};

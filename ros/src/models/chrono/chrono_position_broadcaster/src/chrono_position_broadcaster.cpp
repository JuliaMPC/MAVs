#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>

void poseCallback(const geometry_msgs::Pose& msg){

}

int main(int argc, char** argv){
  //ros::Rate rate(100.0);
  ros::init(argc, argv, "tf_position_broadcaster");

  ros::NodeHandle node;
  double x,y,z,r,p,yaw;


  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  while(1){
    node.getParam("vehicle/chrono/state/x", x) ;
    node.getParam("vehicle/chrono/state/yVal",y);
    node.getParam("hmmwv_chrono/X0/z",z);
    node.getParam("vehicle/chrono/state/psi",yaw); //in radians
    node.getParam("vehicle/chrono/state/theta",p); //in radians
    node.getParam("vehicle/chrono/state/phi",r); //in radians
    transform.setOrigin( tf::Vector3(x,y,z) );
    q.setRPY(r,p,y);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"map", "base_link"));
  }
  return 0;
};

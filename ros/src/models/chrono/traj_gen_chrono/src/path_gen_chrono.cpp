#include <iostream>
#include <vector>
// ROS library
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nloptcontrol_planner/Trajectory.h"
// Chrono library
#include "chrono/core/ChFileutils.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

using namespace chrono;

int main(int argc, char **argv) {

    // initialize rosnode
    ros::init(argc, argv, "Reference");
    // create node handle
    ros::NodeHandle node;

    std::string planner_namespace;
    node.getParam("system/planner",planner_namespace);
    ros::Publisher pub = node.advertise<nloptcontrol_planner::Trajectory>(planner_namespace + "/control", 10);
    // ros::Publisher pub = node.advertise<nloptcontrol_planner::Trajectory>("/control", 10);

    nloptcontrol_planner::Trajectory control_info;
    int control_num = 2;
    std::vector<double> control_t(control_num,0.0);
    std::vector<double> control_sa(control_num,0.0);
    std::vector<double> control_ux(control_num,0.0);
    control_info.x = std::vector<double>(control_num,0.0);
    control_info.y = std::vector<double>(control_num,0.0);
    control_info.psi = std::vector<double>(control_num,0.0);
    control_info.ux = std::vector<double>(control_num,0.0);

    ros::Rate loop_rate(0.1);

    control_info.x = {26.0000, 50};

    int count = 0;
    while (ros::ok()) {
        count++;
        if (count % 2){
            ROS_INFO("Swithed to right lane");
            control_info.y =  {-120.0000, -120.0000};
        }
        else {
            ROS_INFO("Swithed to left lane");
            control_info.y =  {-122.0000, -122.0000};
        }

        control_info.ux[0] = 10;
        pub.publish(control_info);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

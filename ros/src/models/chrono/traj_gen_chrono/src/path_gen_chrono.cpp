#include <iostream>
#include <vector>
// ROS library
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nloptcontrol_planner/Control.h"
#include "ros_chrono_msgs/veh_status.h"

// Chrono library
#include "chrono/core/ChFileutils.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

using namespace chrono;

double veh_x_pos;
double veh_y_pos;
double veh_v;

void vehCallback(const ros_chrono_msgs::veh_status::ConstPtr& veh_msgs) {
    veh_v = std::sqrt(pow(veh_msgs->x_v, 2) + pow(veh_msgs->y_v, 2));
    veh_x_pos = veh_msgs->x_pos;
    veh_y_pos = veh_msgs->y_pos;
}

int main(int argc, char **argv) {
    
    // initialize rosnode
    ros::init(argc, argv, "Reference");
    // create node handle
    ros::NodeHandle node;

    std::string planner_namespace;
    node.getParam("system/planner",planner_namespace);
    ros::Publisher pub = node.advertise<nloptcontrol_planner::Control>(planner_namespace + "/control", 10);
    // ros::Publisher pub = node.advertise<nloptcontrol_planner::Control>("/control", 10);

    ros::Subscriber vehicleinfo_sub = node.subscribe("/vehicleinfo", 100, vehCallback);

    std::vector<double> path_1_x, path_1_y, path_2_x, path_2_y;
    node.getParam("case/path/path_1/x", path_1_x);
    node.getParam("case/path/path_1/y", path_1_y);
    node.getParam("case/path/path_2/x", path_2_x);
    node.getParam("case/path/path_2/y", path_2_y);

    double switch_rate, control_vx;
    node.getParam("case/path/switch_rate", switch_rate);
    node.getParam("case/goal/v", control_vx);

    nloptcontrol_planner::Control control_info;
    control_info.vx = std::vector<double> {control_vx};
    ros::Rate loop_rate(switch_rate);

    int count = 0;
    while (ros::ok()) {
        count++;
        if (count % 2){
            ROS_INFO("Swithed to first lane");
            control_info.x =  path_1_x;
            control_info.y =  path_1_y;
        }
        else {
            ROS_INFO("Swithed to second lane");
            control_info.x =  path_2_x;
            control_info.y =  path_2_y;
        }

        pub.publish(control_info);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

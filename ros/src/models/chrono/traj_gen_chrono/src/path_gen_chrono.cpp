#include <iostream>
#include <vector>
// ROS library
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nloptcontrol_planner/Control.h"
// Chrono library
#include "chrono/core/ChFileutils.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

std::string path_file("MAVs/ros/src/models/chrono/ros_chrono/src/data/vehicle/paths/ISO_double_lane_change.txt");

using namespace chrono;

int main(int argc, char **argv) {
    
    // initialize rosnode
    ros::init(argc, argv, "Reference");
    // create node handle
    ros::NodeHandle node;

    ros::Publisher pub = node.advertise<nloptcontrol_planner::Control>("/control", 10);

    auto path = ChBezierCurve::read(path_file);
    nloptcontrol_planner::Control control_info;
    int control_num = 2;
    std::vector<double> control_t(control_num,0.0);
    std::vector<double> control_sa(control_num,0.0);
    std::vector<double> control_vx(control_num,0.0);
    control_info.x = std::vector<double>(control_num,0.0);
    control_info.y = std::vector<double>(control_num,0.0);
    control_info.psi = std::vector<double>(control_num,0.0);
    control_info.vx = std::vector<double>(control_num,0.0);

    ros::Rate loop_rate(0.1);

    control_info.x = {26.0000, 50};

    int count = 0;
    while (ros::ok()) {
        ROS_INFO("Hello world");

        // if (count < path->getNumPoints()-1) {
        //     ChVector<> pos_1 = path->getPoint(count);
        //     ChVector<> pos_2 = path->getPoint(count+1);

        //     control_info.x = {pos_1[0], pos_2[0]};
        //     control_info.y = {pos_1[1], pos_2[1]};

        //     count++;
        // }

        count++;
        if (count % 2)
            control_info.y =  {-125.0000, -125.0000};
        else
            control_info.y =  {-120.0000, -120.0000};

        control_info.vx[0] = 12;

        ros::spinOnce();
        pub.publish(control_info);
        loop_rate.sleep();
    }
    return 0;
}

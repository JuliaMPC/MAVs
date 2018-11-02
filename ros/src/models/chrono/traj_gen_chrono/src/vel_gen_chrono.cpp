#include "ros/ros.h"
#include <ros/console.h>
#include "std_msgs/String.h"
#include <sstream>
#include <vector>
#include "nloptcontrol_planner/Control.h"

double default_loop_rate = 0.2;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vel_gen_chrono");

    ros::NodeHandle node;
    std::vector<double> x;
    std::vector<double> y;

    std::string planner_namespace;
    node.getParam("system/planner", planner_namespace);

    ros::Publisher pub = node.advertise<nloptcontrol_planner::Control>(planner_namespace+"/control", 10);
    nloptcontrol_planner::Control control_msgs;
    
    int msgs_len = 10;
    std::vector<double> control_t(msgs_len, 0.0);
    std::vector<double> control_sa(msgs_len, 0.0);
    std::vector<double> control_vx(msgs_len, 0.0);
    control_msgs.x = std::vector<double>(msgs_len, 0.0);
    control_msgs.y = std::vector<double>(msgs_len, 0.0);
    control_msgs.psi = std::vector<double>(msgs_len, 0.0);

    ros::Rate loop_rate(default_loop_rate);

    int num = 1;
    while (ros::ok()) {
        for (int i = 0; i < msgs_len; i++)
            control_t[i] = i / (default_loop_rate * msgs_len);
        
        if (num) {
            for (int i = 0; i < msgs_len; i++)
                control_vx[i] = 10.0;
        }
        else{
            for (int i = 0; i < msgs_len; i++)
                control_vx[i] = 0.0;
        }

        num = !num;

        control_msgs.t = control_t;
        control_msgs.sa = control_sa;
        control_msgs.vx = control_vx;

        pub.publish(control_msgs);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

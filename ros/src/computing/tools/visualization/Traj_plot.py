#!/usr/bin/env python 
import numpy as np
from matplotlib import pyplot as plt
import rospy
from ros_chrono_msgs.msg import veh_status
from nloptcontrol_planner.msg import Trajectory
#from std_msgs.msg import State

def plot_x(msg):
    global x_
    global y_
    global counter
    if counter % 10 == 0:
        #stamp = msg.header.stamp
        #time = stamp.secs + stamp.nsecs * 1e-9
        x_ = np.append(x_,msg.x_pos)
        y_ = np.append(y_,msg.y_pos)
        plt.plot(x_, y_,'b')
        plt.axis("equal")
        plt.draw()
        plt.pause(0.00000000001)

    counter += 1

def plot_opt(msg):
    global counter
    if counter % 10 == 0:
        plt.plot(msg.x,msg.y,'r')
        plt.axis("equal")
        plt.draw()
        plt.pause(0.00000000001)

    counter += 1

if __name__ == '__main__':
    counter = 0
    x_ = 0
    y_ = 0
    rospy.init_node("plotter")
    rospy.Subscriber("chrono/vehicleinfo", veh_status, plot_x)
    rospy.Subscriber("nlopcontrol_planner/control", Trajectory, plot_opt)
    plt.ion()
    plt.show()
    rospy.spin()

#!/usr/bin/env python 
import numpy as np
from matplotlib import pyplot as plt
import rospy
from ros_chrono_msgs.msg import veh_status
from nloptcontrol_planner.msg import Trajectory
#from std_msgs.msg import State



def plot_1(msg):
    global counter
    if counter % 10 == 0:
        #stamp = msg.header.stamp
        #time = stamp.secs + stamp.nsecs * 1e-9
        #plt.plot(msg.x_pos, msg.y_pos)
        plt.plot(msg.x, msg.y,"r")
        plt.axis("equal")
        plt.draw()
        plt.pause(0.00000000001)
        plt.hold(True)
	counter = 0

    counter += 1

def plot_2(msg):
    global counter2
    if counter2 % 10 == 0:
        #stamp = msg.header.stamp
        #time = stamp.secs + stamp.nsecs * 1e-9
        plt.plot(msg.x_pos, msg.y_pos, "g*")
        #plt.plot(msg.x, msg.y)
        plt.axis("equal")
        plt.draw()
        plt.pause(0.00000000001)
	counter2 = 0

    counter2 += 1


if __name__ == '__main__':
    counter = 0
    counter2 = 0
    rospy.init_node("plotter")
    rospy.Subscriber("nlopcontrol_planner/control", Trajectory, plot_1)
    rospy.Subscriber("chrono/vehicleinfo", veh_status, plot_2)
    plt.ion()
    plt.show()
    rospy.spin()

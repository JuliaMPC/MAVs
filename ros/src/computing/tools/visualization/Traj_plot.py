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
    global x_1
    global y_1
    global x_2
    global y_2
    global x_3
    global y_3
    global counter
    if counter % 10 == 0:
        plt.subplot(221)
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.title("Predicted and actual trajectory")
        x_ = np.append(x_,msg.x_pos)
        y_ = np.append(y_,msg.y_pos)
        plt.plot(x_, y_,'b')
        plt.axis("equal")
        plt.ylim(msg.y_pos - 5, msg.y_pos + 5) # Moving window
        plt.xlim(msg.x_pos - 5, msg.x_pos + 5) # Moving window

        plt.subplot(222)
        plt.xlabel("Time [s]")
        plt.ylabel("Steering Agnle [degree]")
        plt.title("Predicted and actual Steering Angle")
        x_1 = np.append(x_1,msg.t_chrono)
        y_1 = np.append(y_1,msg.sa)
        plt.plot(x_1, y_1,'b')
        plt.xlim(msg.t_chrono - 5, msg.t_chrono + 5) # Moving window

        plt.subplot(223)
        plt.xlabel("Time [s]")
        plt.ylabel("x_a [m/s^2]")
        plt.title("Predicted and actual longitudinal acceleration")
        x_2 = np.append(x_2,msg.t_chrono)
        y_2 = np.append(y_2,msg.x_a)
        plt.plot(x_2, y_2,'b')
        plt.xlim(msg.t_chrono - 5, msg.t_chrono + 5) # Moving window

        plt.subplot(224)
        plt.xlabel("Time [s]")
        plt.ylabel("ux [m/s]")
        plt.title("Predicted and actual longitudinal acceleration")
        x_3 = np.append(x_3,msg.t_chrono)
        y_3 = np.append(y_3,msg.x_v)
        plt.plot(x_3, y_3,'b')
        plt.xlim(msg.t_chrono - 5, msg.t_chrono + 5) # Moving window

        plt.pause(0.00000001)

    counter += 1

def plot_opt(msg):
    global counter
    if counter % 10 == 0:
        plt.subplot(221)
        plt.plot(msg.x,msg.y,'r')
        plt.axis("equal")

        plt.subplot(222)
        plt.plot(msg.t,msg.sa,'r')

        plt.subplot(223)
        plt.plot(msg.t,msg.ax,'r')
        plt.draw()

        plt.subplot(224)
        plt.plot(msg.t,msg.ux,'r')
        plt.draw()

        plt.pause(0.00000001)

    counter += 1


if __name__ == '__main__':
    counter = 0
    x_ = 0
    y_ = 0
    x_1 = 0
    y_1 = 0
    x_2 = 0
    y_2 = 0
    x_3 = 0
    y_3 = 0
    rospy.init_node("plotter")
    rospy.Subscriber("chrono/vehicleinfo", veh_status, plot_x)
    rospy.Subscriber("nlopcontrol_planner/control", Trajectory, plot_opt)
    plt.ion()
    plt.show()
    rospy.spin()

 #  Tips for plt.title's format
 #    {'fontsize': rcParams['axes.titlesize'],
 #      'fontweight' : rcParams['axes.titleweight'],
 #      'verticalalignment': 'baseline',
 #      'horizontalalignment': loc}

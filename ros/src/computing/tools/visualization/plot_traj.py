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
    global x_pos
    global y_pos
    global t_chrono
    global counter

    x_ = np.append(x_,msg.x_pos)
    y_ = np.append(y_,msg.y_pos)
    x_pos = msg.x_pos
    y_pos = msg.y_pos

    x_1 = np.append(x_1,msg.t_chrono)
    y_1 = np.append(y_1,msg.sa)
    t_chrono = msg.t_chrono

    x_2 = np.append(x_2,msg.t_chrono)
    y_2 = np.append(y_2,msg.x_a)

    x_3 = np.append(x_3,msg.t_chrono)
    y_3 = np.append(y_3,msg.x_v)


def plot_opt(msg):
    global counter
    global opt_x
    global opt_y
    global opt_t
    global opt_ax
    global opt_sa

    opt_x = msg.x
    opt_y = msg.y
    opt_t = msg.t
    opt_sa = msg.sa
    opt_ax = msg.ax

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
    opt_x = 0
    opt_y = 0
    opt_t = 0
    opt_sa = 0
    opt_ax = 0
    x_pos = 0
    y_pos = 0
    t_chrono = 0
    rospy.init_node("plotter")
    rospy.Subscriber("chrono/vehicleinfo", veh_status, plot_x)
    rospy.Subscriber("nlopcontrol_planner/control", Trajectory, plot_opt)


    rate = rospy.Rate(50) # 50hz

    while not rospy.is_shutdown():

        plt.subplot(221)
        plt.plot(opt_x,opt_y,'r')

        plt.subplot(222)
        plt.plot(opt_t,opt_sa,'r')

        plt.subplot(223)
        plt.plot(opt_t,opt_ax,'r')

        plt.subplot(221)
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.title("Predicted and actual trajectory")
        plt.plot(x_, y_,'b')
        plt.axis("equal")
        plt.ylim(y_pos - 5, y_pos + 5) # Moving window
        plt.xlim(x_pos - 5, x_pos + 5) # Moving window

        plt.subplot(222)
        plt.xlabel("Time [s]")
        plt.ylabel("Steering Agnle [degree]")
        plt.title("Predicted and actual Steering Angle")
        plt.plot(x_1, y_1,'b')
        plt.draw()
        plt.xlim(t_chrono - 2, t_chrono + 2) # Moving window

        plt.subplot(223)
        plt.xlabel("Time [s]")
        plt.ylabel("x_a [m/s^2]")
        plt.title("Predicted and actual longitudinal acceleration")
        plt.plot(x_2, y_2,'b')
        plt.xlim(t_chrono - 2, t_chrono + 2) # Moving window


        plt.subplot(224)
        plt.xlabel("Time [s]")
        plt.ylabel("ux [m/s]")
        plt.title("Predicted and actual longitudinal acceleration")        
        plt.plot(x_3, y_3,'b')

        plt.draw()
        plt.pause(0.00000001)
        rate.sleep()

    plt.ion()
    plt.show()
    rospy.spin()
 #  Tips for plt.title's format
 #    {'fontsize': rcParams['axes.titlesize'],
 #      'fontweight' : rcParams['axes.titleweight'],
 #      'verticalalignment': 'baseline',
 #      'horizontalalignment': loc}

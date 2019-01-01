#!/usr/bin/env python
import numpy as np
from matplotlib import pyplot as plt
import rospy
from mavs_msgs.msg import state, control
from nloptcontrol_planner.msg import Trajectory
#from std_msgs.msg import State

def plot_x(msg):
    global x_
    global y_
    global x_pos
    global y_pos

    x_ = np.append(x_,msg.x)
    y_ = np.append(y_,msg.y)
    x_pos = msg.x
    y_pos = msg.y

def plot_opt(msg):
    global counter
    global opt_x
    global opt_y
    global opt_x_past
    global opt_y_past

    opt_x_past = opt_x
    opt_y_past = opt_y
    opt_x = msg.x
    opt_y = msg.y
    opt_t = msg.t

if __name__ == '__main__':
    counter = 0
    x_ = 0
    y_ = 0
    opt_x = 0
    opt_y = 0
    opt_x_past = 0
    opt_y_past = 0
    x_pos = 0
    y_pos = 0
    t_chrono = 0
    rospy.init_node("plotter5")
    flag_state = rospy.Subscriber("state", state, plot_x)
    flag_traj = rospy.Subscriber("nloptcontrol_planner/control", Trajectory, plot_opt)


    rate = rospy.Rate(400) # 50hz

    while not rospy.is_shutdown():

        plt.figure(1)
        plt.plot(opt_x,opt_y,'r', opt_x_past, opt_y_past, 'r--')
        # plt.legend(['Predicted Trajectory - Latest'], ['Predicted Trajectory - Last'])
        plt.draw()
        plt.pause(0.1)

        try:
            plt.figure(1)
            plt.clf()
            plt.xlabel("X [m]")
            plt.ylabel("Y [m]")
            plt.title("Predicted and actual trajectory")
            plt.plot(x_, y_,'b')
            plt.ylim(y_pos - 5, y_pos + 5) # Moving window
            plt.xlim(x_pos - 5, x_pos + 5) # Moving window
            plt.draw()
            plt.pause(0.1)
        except Exception:
            pass

        rate.sleep()

    plt.ion()
    plt.show()
    rospy.spin()
 #  Tips for plt.title's format
 #    {'fontsize': rcParams['axes.titlesize'],
 #      'fontweight' : rcParams['axes.titleweight'],
 #      'verticalalignment': 'baseline',
 #      'horizontalalignment': loc}

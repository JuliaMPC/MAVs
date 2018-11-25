#!/usr/bin/env python
import numpy as np
from matplotlib import pyplot as plt
import rospy
from mavs_msgs.msg import state, control
from nloptcontrol_planner.msg import Trajectory
#from std_msgs.msg import State

def plot_x(msg):
    global t
    global ax
    global t_chrono
    global ax_max
    global ax_min

    t = np.append(t,msg.t)
    ax = np.append(ax,msg.ax)
    t_chrono = msg.t

def plot_opt(msg):
    global counter
    global opt_t
    global opt_ax
    global opt_t_past
    global opt_ax_past

    opt_t_past = opt_t
    opt_ax_past = opt_ax
    opt_t = msg.t
    opt_ax = msg.ax

if __name__ == '__main__':
    counter = 0
    t = 0
    t_limit = 0
    ax = 0
    opt_t = 0
    opt_ax = 0
    opt_t_past = 0
    opt_ax_past = 0
    ax_min_temp = 0
    ax_max_temp = 0
    ax_min = 0
    ax_max = 0
    t_chrono = 0
    rospy.init_node("plotter7")
    flag_state = rospy.Subscriber("state", state, plot_x)
    flag_traj = rospy.Subscriber("nlopcontrol_planner/control", Trajectory, plot_opt)


    rate = rospy.Rate(400) # 50hz

    while not rospy.is_shutdown():
        ax_min_temp = rospy.get_param('/vehicle/nloptcontrol_planner/ax_min')
        ax_max_temp = rospy.get_param('/vehicle/nloptcontrol_planner/ax_max')
        ax_min = np.append(ax_min, ax_min_temp)
        ax_max = np.append(ax_max, ax_max_temp)
        t_limit = np.arange(0.0, 3*(len(ax_max))/100.0, 0.03)


        plt.figure(1)
        plt.plot(opt_t,opt_ax,'r', opt_t_past, opt_ax_past, 'r--')
        plt.xlim(t_chrono - 5, t_chrono + 5) # Moving window
	plt.ylim(ax_min_temp*1.5, ax_max_temp*1.5)
        # plt.legend((line1, line2),('Predicted Acceleration - Latest', 'Predicted Acceleration - Last'))
        plt.draw()
        plt.pause(0.00000001)
        try:
            plt.figure(1)
            plt.clf()
            plt.xlabel("t [s]")
            plt.ylabel("acceleration [m/s^2]")
            plt.title("Predicted and actual longitudinal acceleration")
            plt.plot(t, ax,'b', t_limit, ax_min, 'y--', t_limit, ax_max, 'y--')
            plt.xlim(t_chrono - 5, t_chrono + 5) # Moving window
            plt.ylim(ax_min_temp*1.5, ax_max_temp*1.5)
            plt.draw()
            plt.pause(0.00000001)
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

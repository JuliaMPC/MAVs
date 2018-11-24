#!/usr/bin/env python
import numpy as np
from matplotlib import pyplot as plt
import rospy
from mavs_msgs.msg import state, control
from nloptcontrol_planner.msg import Trajectory
#from std_msgs.msg import State

def plot_x(msg):
    global t
    global sa
    global sa_temp
    global t_chrono

    t = np.append(t,msg.t)
    sa = np.append(sa,msg.sa)
    t_chrono = msg.t
    sa_temp = msg.sa

def plot_opt(msg):
    global counter
    global opt_t
    global opt_t_past
    global opt_sa
    global opt_sa_past

    opt_t_past = opt_t
    opt_sa_past = opt_sa
    opt_sa = msg.sa
    opt_t = msg.t

if __name__ == '__main__':
    counter = 0
    t = 0
    t_chrono = 0
    opt_t = 0
    opt_sa = 0
    opt_t_past = 0
    opt_sa_past = 0
    sa = 0
    sa_temp = 0
    rospy.init_node("plotter6")
    flag_state = rospy.Subscriber("state", state, plot_x)
    flag_traj = rospy.Subscriber("nlopcontrol_planner/control", Trajectory, plot_opt)


    rate = rospy.Rate(400) # 50hz

    while not rospy.is_shutdown():

        plt.figure(1)
        plt.plot(opt_t,opt_sa,'r', opt_t_past, opt_sa_past, 'r--')
        # plt.legend(['Predicted Steering Angle - Latest'], ['Predicted Steering Angle - Last'])
        plt.draw()
        plt.pause(0.00000001)
        try:
            plt.figure(1)
            plt.clf()
            plt.xlabel("t [s]")
            plt.ylabel("Steering Angle [degree]")
            plt.title("Predicted and actual steering angle")
            plt.plot(t, sa,'b')
            plt.xlim(t_chrono - 5, t_chrono + 5) # Moving window
            plt.ylim(-0.1, 0.1) # Moving window
            plt.draw()
            plt.pause(0.00000001)
        except Exception:
            pass


        rate.sleep()

    plt.ion()
    plt.show()
    rospy.spin()
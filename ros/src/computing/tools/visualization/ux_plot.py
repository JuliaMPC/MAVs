#!/usr/bin/env python
import numpy as np
from matplotlib import pyplot as plt
import rospy
from mavs_msgs.msg import state, control
from nloptcontrol_planner.msg import Trajectory
#from std_msgs.msg import State

def plot_x(msg):
    global t
    global ux
    global t_chrono

    t = np.append(t,msg.t)
    ux = np.append(ux,msg.ux)
    t_chrono = msg.t

if __name__ == '__main__':
    counter = 0
    t = 0
    t_chrono = 0
    ux = 0
    rospy.init_node("plotter8")
    flag_state = rospy.Subscriber("state", state, plot_x)

    rate = rospy.Rate(400) # 50hz

    while not rospy.is_shutdown():

        if flag_state:
            try:
                plt.figure(1)
                plt.clf()
                plt.xlabel("t [s]")
                plt.ylabel("velocity [m/s]")
                plt.title("Actual velocity")
                plt.plot(t, ux,'b')
                # plt.legend(['Actual velocity'])
                plt.xlim(t_chrono - 5, t_chrono + 5) # Moving window
                plt.draw()
                plt.pause(0.00000001)
            except Exception:
                pass

        rate.sleep()

    plt.ion()
    plt.show()
    rospy.spin()
    
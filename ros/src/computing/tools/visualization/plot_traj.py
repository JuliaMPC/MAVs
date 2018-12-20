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
    global ax_max
    global ax_min

    x_ = np.append(x_,msg.x)
    y_ = np.append(y_,msg.y)
    x_pos = msg.x
    y_pos = msg.y
    x_ = np.array(x_)
    y_ = np.array(y_)

    x_1 = np.append(x_1,msg.t)
    y_1 = np.append(y_1,msg.sa)
    t_chrono = msg.t
    x_1 = np.array(x_1)
    y_1 = np.array(y_1)

    x_2 = np.append(x_2,msg.t)
    y_2 = np.append(y_2,msg.ax)
    x_2 = np.array(x_2)
    y_2 = np.array(y_2)

    x_3 = np.append(x_3,msg.t)
    y_3 = np.append(y_3,msg.ux)
    x_3 = np.array(x_3)
    y_3 = np.array(y_3)

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
    ax_min = 0
    ax_max = 0
    t_chrono = 0
    rospy.init_node("plotter")
    flag_state = rospy.Subscriber("state", state, plot_x)
    flag_traj = rospy.Subscriber("nlopcontrol_planner/control", Trajectory, plot_opt)


    rate = rospy.Rate(400) # 50hz

    while not rospy.is_shutdown():
        ax_min = rospy.get_param('/vehicle/nloptcontrol_planner/ax_min')
        ax_max = rospy.get_param('/vehicle/nloptcontrol_planner/ax_max')

        if flag_traj:
            pass
            plt.subplot(221)
            plt.plot(opt_x,opt_y,'r')

            plt.subplot(222)
            plt.plot(opt_t,opt_sa,'r')

            plt.subplot(223)
            plt.plot(opt_t,opt_ax,'r')
            plt.draw()
            plt.pause(0.00000001)

        if flag_state:
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
            plt.xlim(t_chrono - 2, t_chrono + 2) # Moving window

            plt.subplot(223)
            plt.xlabel("Time [s]")
            plt.ylabel("x_a [m/s^2]")
            plt.title("Predicted and actual longitudinal acceleration")
            plt.plot(x_2, y_2,'b')
            plt.plot(t_chrono, ax_min, 'r*')
            plt.plot(t_chrono, ax_max, 'r*')
            plt.xlim(t_chrono - 2, t_chrono + 2) # Moving window
            plt.ylim(ax_min*1.5, ax_max*1.5)

            plt.subplot(224)
            plt.xlabel("Time [s]")
            plt.ylabel("ux [m/s]")
            plt.title("Predicted and actual longitudinal acceleration")
            plt.plot(x_3, y_3,'b')
            plt.draw()
            plt.pause(0.00000001)

        # rospy.loginfo("x_####: %f", *x_)
        # rospy.loginfo("y_####: %f", *y_)

        # Print x array start
        print('t_chrono')
        print(' '.join(map(str, x_1)))  
        # print('\n'.join(map(str, x_))) 
        print"in new line"
        # Print x array end  

        # # Print size
        # print('x size: %d', len(x_))
        # print('y size: %d', len(y_))
        # print('t size: %d', len(x_1))
        # print('sa size: %d', len(y_1))
        # print('t size: %d', len(x_2))
        # print('ax size: %d', len(y_2))
        # print('t size: %d', len(x_3))
        # print('ux size: %d', len(y_3))
        # print('\n')
        # # Print size

        rate.sleep()

    plt.ion()
    plt.show()
    rospy.spin()
 #  Tips for plt.title's format
 #    {'fontsize': rcParams['axes.titlesize'],
 #      'fontweight' : rcParams['axes.titleweight'],
 #      'verticalalignment': 'baseline',
 #      'horizontalalignment': loc}

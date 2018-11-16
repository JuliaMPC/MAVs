#!/usr/bin/env python 
import numpy as np
from matplotlib import pyplot as plt
import rospy
from nloptcontrol_planner.msg import Optimization

def plot_tsolve(msg):
    global solve_num

    solve_num += 1
    plt.figure(2)
    plt.title("Nonlinear Optimization Controller Solve Time")
    plt.xlabel("Number of optimization calculations")
    plt.ylabel("Sove Time [s]")
    plt.ylim(0.3,0.7)
    # if counter % 10 == 0:
	plt.plot(solve_num,msg.tSolve,'g*')
	plt.plot(solve_num,msg.tSolve,'r*')
    # plt.axis("equal")
    plt.draw()
    plt.pause(0.00001)

    # counter += 1

if __name__ == '__main__':
    solve_num = 0
    rospy.init_node("plotter2")
    rospy.Subscriber("nlopcontrol_planner/opt", Optimization, plot_tsolve)
    plt.ion()
    plt.show()
    rospy.spin()

 #  Tips for plt.title's format
 #    {'fontsize': rcParams['axes.titlesize'],
 # 		'fontweight' : rcParams['axes.titleweight'],
 # 		'verticalalignment': 'baseline',
 # 		'horizontalalignment': loc}



    

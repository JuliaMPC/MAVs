#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import matplotlib.animation as animation
import rospy
from mavs_msgs.msg import state, control
from nloptcontrol_planner.msg import Trajectory
from nloptcontrol_planner.msg import Optimization

class DataPlotter():
    def __init__(self):
        # time data
        self.t_actual = np.array([], dtype=np.float64)
        self.t_traj = np.array([], dtype=np.float64)
        self.t_traj_last = np.array([], dtype=np.float64)
        
        # position data
        self.x_actual = np.array([], dtype=np.float64)
        self.y_actual = np.array([], dtype=np.float64)
        self.x_traj = np.array([], dtype=np.float64)
        self.y_traj = np.array([], dtype=np.float64)
        self.x_traj_last = np.array([], dtype=np.float64)
        self.y_traj_last = np.array([], dtype=np.float64)

        # steering angle data
        self.sa_actual = np.array([], dtype=np.float64)
	self.sa_actual_real = np.array([], dtype=np.float64)
	self.sa_cumsum = 0
        self.sa_traj = np.array([], dtype=np.float64)
        self.sa_traj_last = np.array([], dtype=np.float64)
        self.sa_upper_limit = np.array([], dtype=np.float64)
        self.sa_lower_limit = np.array([], dtype=np.float64)

        # longitudinal velocity data
        self.ux_actual = np.array([], dtype=np.float64)
        self.ux_traj = np.array([], dtype=np.float64)
        self.ux_traj_last = np.array([], dtype=np.float64)

        # longitudinal acceleration data
        self.ax_actual = np.array([], dtype=np.float64)
        self.ax_actual_real = np.array([], dtype=np.float64)
	self.ax_cumsum = 0
        self.ax_traj = np.array([], dtype=np.float64)
        self.ax_traj_last = np.array([], dtype=np.float64)
        self.ax_upper_limit = np.array([], dtype=np.float64)
        self.ax_lower_limit = np.array([], dtype=np.float64)

        # solving time data
        self.solve_num = np.array([], dtype=np.uint32)
        self.solve_num_pass = np.array([], dtype=np.uint32)
        self.solve_num_fail = np.array([], dtype=np.uint32)
        self.solve_time = np.array([], dtype=np.float64)
        self.solve_time_pass = np.array([], dtype=np.float64)
        self.solve_time_fail = np.array([], dtype=np.float64)
        self.solve_time_limit = np.array([], dtype=np.float64)

        # setup matplot figure
        self.fig = plt.figure(figsize=(12, 10))
        self.fig.subplots_adjust(top=0.95, bottom=0.075, left=0.075, right=0.95)
        self.pos_subplot = self.fig.add_subplot(1,2,1)
        self.sa_subplot = self.fig.add_subplot(4,2,2)
        self.ux_subplot = self.fig.add_subplot(4,2,4)
        self.ax_subplot = self.fig.add_subplot(4,2,6)
        self.tSolve_subplot = self.fig.add_subplot(4,2,8)

        # axis band
        self.t_band = 10
        self.x_band = 20
        self.y_band = 50
        self.sa_band = 0.2
        self.ux_band = 20
        self.ax_band = 5
        self.solve_num_band = 20

        self.line_pos_actual = Line2D([], [], color='green', linewidth=1.5, linestyle='-')
        self.line_pos_traj = Line2D([], [], color='red', linewidth=1, linestyle='--')
        self.line_pos_traj_last = Line2D([], [], color='orange', linewidth=1, linestyle='--')
        self.pos_subplot.add_line(self.line_pos_actual)
        self.pos_subplot.add_line(self.line_pos_traj)
        self.pos_subplot.add_line(self.line_pos_traj_last)
        self.pos_subplot.legend(['actual path', 'planned path', 'last planned path'], fontsize='x-small')
        self.pos_subplot.set_xlabel('x (m)', fontsize='larger')
        self.pos_subplot.set_ylabel('y (m)', fontsize='larger')
        self.pos_subplot.tick_params('both', labelsize='small')
        self.pos_subplot.grid(True)
        self.pos_subplot.axis('equal')
        self.pos_subplot.set_xlim(-self.x_band/2.0,self.x_band/2.0)
        self.pos_subplot.set_ylim(-self.y_band/2.0,self.y_band/2.0)

        self.line_sa_actual = Line2D([], [], color='green', linewidth=2, linestyle='-')
        self.line_sa_traj = Line2D([], [], color='red', linewidth=1.5, linestyle='--')
        self.line_sa_traj_last = Line2D([], [], color='orange', linewidth=1.5, linestyle='--')
        self.sa_subplot.add_line(self.line_sa_actual)
        self.sa_subplot.add_line(self.line_sa_traj)
        self.sa_subplot.add_line(self.line_sa_traj_last)
        self.sa_subplot.legend(['actual', 'planned', 'last planned'], fontsize='x-small')
        self.sa_subplot.set_xlabel('time (s)', fontsize='larger', verticalalignment='center')
        self.sa_subplot.set_ylabel('sa (rad)', fontsize='larger')
        self.sa_subplot.tick_params('both', labelsize='x-small')
        self.sa_subplot.grid(True)
        self.sa_subplot.set_xlim(0,self.t_band)
        self.sa_subplot.set_ylim(-self.sa_band/2.0,self.sa_band/2.0)

        self.line_ux_actual = Line2D([], [], color='green', linewidth=2, linestyle='-')
        self.line_ux_traj = Line2D([], [], color='red', linewidth=1.5, linestyle='--')
        self.line_ux_traj_last = Line2D([], [], color='orange', linewidth=1.5, linestyle='--')
        self.ux_subplot.add_line(self.line_ux_actual)
        self.ux_subplot.add_line(self.line_ux_traj)
        self.ux_subplot.add_line(self.line_ux_traj_last)
        self.ux_subplot.legend(['actual', 'planned', 'last planned'], fontsize='x-small')
        self.ux_subplot.set_xlabel('time (s)', fontsize='larger', verticalalignment='center')
        self.ux_subplot.set_ylabel('ux (m/s)', fontsize='larger')
        self.ux_subplot.tick_params('both', labelsize='x-small')
        self.ux_subplot.grid(True)
        self.ux_subplot.set_xlim(0,self.t_band)
        self.ux_subplot.set_ylim(-self.ux_band/2.0,self.ux_band/2.0)

        self.line_ax_actual = Line2D([], [], color='green', linestyle='-')
        self.line_ax_traj = Line2D([], [], color='red', linewidth=1.5, linestyle='--')
        self.line_ax_traj_last = Line2D([], [], color='orange', linewidth=1.5, linestyle='--')
        self.ax_subplot.add_line(self.line_ax_actual)
        self.ax_subplot.add_line(self.line_ax_traj)
        self.ax_subplot.add_line(self.line_ax_traj_last)
        self.ax_subplot.legend(['actual', 'planned', 'last planned'], fontsize='x-small')
        self.ax_subplot.set_xlabel('time (s)', fontsize='larger', verticalalignment='center')
        self.ax_subplot.set_ylabel('ax (m/s)', fontsize='larger')
        self.ax_subplot.tick_params('both', labelsize='x-small')
        self.ax_subplot.grid(True)
        self.ax_subplot.set_xlim(0,self.t_band)
        self.ax_subplot.set_ylim(-self.ax_band/2.0,self.ax_band/2.0)

        self.line_solve_time = Line2D([], [], marker='*', color='green', linestyle='')
        self.line_solve_time_pass = Line2D([], [], marker='*', color='green', linestyle='')
        self.line_solve_time_fail = Line2D([], [], marker='*', color='red', linestyle='')
        self.line_solve_time_limit = Line2D([], [], color='red', linewidth=1.5, linestyle='--')
        # self.tSolve_subplot.add_line(self.line_solve_time)
        self.tSolve_subplot.add_line(self.line_solve_time_pass)
        self.tSolve_subplot.add_line(self.line_solve_time_fail)
        self.tSolve_subplot.add_line(self.line_solve_time_limit)
	self.tSolve_subplot.legend(['optimal', 'non-optimal', 'solve time limit'], fontsize='x-small')
        self.tSolve_subplot.set_xlabel('number of optimization calculations', fontsize='larger', verticalalignment='center')
        self.tSolve_subplot.set_ylabel('solve time (s)', fontsize='larger')
        self.tSolve_subplot.tick_params('both', labelsize='x-small')
        self.tSolve_subplot.grid(True)
        self.tSolve_subplot.set_xlim(0,self.solve_num_band)
        self.tSolve_subplot.set_ylim(0,1.0)

    def draw_frame(self):
        try:

            self.line_pos_actual.set_data(self.x_actual, self.y_actual)
            self.line_pos_traj.set_data(self.x_traj, self.y_traj)
            self.line_pos_traj_last.set_data(self.x_traj_last, self.y_traj_last)
            if (len(self.x_actual) and len(self.y_actual)):
                self.pos_subplot.set_xlim(self.x_actual[-1]-self.x_band/2, self.x_actual[-1]+self.x_band/2)
                self.pos_subplot.set_ylim(self.y_actual[-1]-self.y_band/2, self.y_actual[-1]+self.y_band/2)

            self.line_sa_actual.set_data(self.t_actual, self.sa_actual)
            self.line_sa_traj.set_data(self.t_traj, self.sa_traj)
            self.line_sa_traj_last.set_data(self.t_traj_last, self.sa_traj_last)

            self.line_ux_actual.set_data(self.t_actual, self.ux_actual)
            self.line_ux_traj.set_data(self.t_traj, self.ux_traj)
            self.line_ux_traj_last.set_data(self.t_traj_last, self.ux_traj_last)
            if (len(self.ux_actual)):
                self.ux_subplot.set_ylim(self.ux_actual[-1]-self.ux_band/2, self.ux_actual[-1]+self.ux_band/2)

            self.line_ax_actual.set_data(self.t_actual, self.ax_actual)
            self.line_ax_traj.set_data(self.t_traj, self.ax_traj)
            self.line_ax_traj_last.set_data(self.t_traj_last, self.ax_traj_last)

            self.line_solve_time.set_data(self.solve_num, self.solve_time)
            self.line_solve_time_pass.set_data(self.solve_num_pass, self.solve_time_pass)
            self.line_solve_time_fail.set_data(self.solve_num_fail, self.solve_time_fail)
            self.line_solve_time_limit.set_data(self.solve_num, self.solve_time_limit)

            # moving time axis
            if (len(self.t_actual) > 0 and self.t_actual[-1] > 10):
                self.sa_subplot.set_xlim(self.t_actual[-1]-self.t_band*3/4, self.t_actual[-1]+self.t_band/4)
                self.ux_subplot.set_xlim(self.t_actual[-1]-self.t_band*3/4, self.t_actual[-1]+self.t_band/4)
                self.ax_subplot.set_xlim(self.t_actual[-1]-self.t_band*3/4, self.t_actual[-1]+self.t_band/4)
                
            if (len(self.solve_num) > 0 and self.solve_num[-1] > 20):
                self.tSolve_subplot.set_xlim(self.t_actual[-1]-10, self.t_actual[-1]+10)

            
            plt.draw()
            plt.pause(0.001) #0.001
        except:
            rospy.loginfo("weird exception")

def stateCallback(msg):
    global data_plotter
    num_avg = 30
    data_plotter.t_actual = np.append(data_plotter.t_actual, msg.t)
    data_plotter.x_actual = np.append(data_plotter.x_actual, msg.x)
    data_plotter.y_actual = np.append(data_plotter.y_actual, msg.y)
    data_plotter.ux_actual = np.append(data_plotter.ux_actual, msg.ux)
    data_plotter.sa_actual_real = np.append(data_plotter.sa_actual_real, msg.sa)
    data_plotter.ax_actual_real = np.append(data_plotter.ax_actual_real, msg.ax)

    if(len(data_plotter.ax_actual_real) > num_avg):
	data_plotter.ax_cumsum = data_plotter.ax_cumsum + data_plotter.ax_actual_real[(len(data_plotter.ax_actual_real)-1)]-data_plotter.ax_actual_real[(len(data_plotter.ax_actual_real)-num_avg-1)]
   	temp = data_plotter.ax_cumsum/num_avg
	data_plotter.ax_actual = np.append(data_plotter.ax_actual, temp)
    else:
	data_plotter.ax_cumsum = data_plotter.ax_cumsum + data_plotter.ax_actual_real[len(data_plotter.ax_actual_real)-1]
	temp = data_plotter.ax_cumsum/len(data_plotter.ax_actual_real)
	data_plotter.ax_actual = np.append(data_plotter.ax_actual, temp)
	
    
    if(len(data_plotter.sa_actual_real) > num_avg):
	data_plotter.sa_cumsum = data_plotter.sa_cumsum + data_plotter.sa_actual_real[(len(data_plotter.sa_actual_real)-1)]-data_plotter.sa_actual_real[(len(data_plotter.sa_actual_real)-num_avg-1)]
   	temp = sum(data_plotter.sa_actual_real[(len(data_plotter.sa_actual_real)-num_avg-1):(len(data_plotter.sa_actual_real)-1)])/num_avg
	data_plotter.sa_actual = np.append(data_plotter.sa_actual, temp)
    else:
	data_plotter.sa_cumsum = data_plotter.sa_cumsum + data_plotter.sa_actual_real[len(data_plotter.sa_actual_real)-1]
	temp = data_plotter.sa_cumsum/len(data_plotter.sa_actual_real)
	data_plotter.sa_actual = np.append(data_plotter.sa_actual, temp)
    

def trajectoryCallback(msg):
    global data_plotter

    data_plotter.t_traj_last = data_plotter.t_traj
    data_plotter.x_traj_last = data_plotter.x_traj
    data_plotter.y_traj_last = data_plotter.y_traj
    data_plotter.sa_traj_last = data_plotter.sa_traj
    data_plotter.ux_traj_last = data_plotter.ux_traj
    data_plotter.ax_traj_last = data_plotter.ax_traj

    data_plotter.t_traj = msg.t
    data_plotter.x_traj = msg.x
    data_plotter.y_traj = msg.y
    data_plotter.sa_traj = msg.sa
    data_plotter.ux_traj = msg.ux
    data_plotter.ax_traj = msg.ax

def optCallback(msg):
    global data_plotter

    if (msg.tSolve < 0.5) & (msg.status == 'Optimal') :
        data_plotter.solve_time_pass = np.append(data_plotter.solve_time_pass, msg.tSolve)
        data_plotter.solve_num_pass = np.append(data_plotter.solve_num_pass, len(data_plotter.solve_num))
    else:
        data_plotter.solve_time_fail = np.append(data_plotter.solve_time_fail, msg.tSolve)
        data_plotter.solve_num_fail = np.append(data_plotter.solve_num_fail, len(data_plotter.solve_num))
    data_plotter.solve_time = np.append(data_plotter.solve_time, msg.tSolve)
    data_plotter.solve_time_limit = np.append(data_plotter.solve_time_limit, 0.5)
    data_plotter.solve_num = np.append(data_plotter.solve_num, len(data_plotter.solve_time))


if __name__ == '__main__':

    # initialize node
    rospy.init_node("visualization_plotter")
    
    # create a data plotter instance
    data_plotter = DataPlotter()

    # setup ROS subscriber
    rospy.Subscriber("state", state, stateCallback)
    rospy.Subscriber("nlopcontrol_planner/control", Trajectory, trajectoryCallback)
    rospy.Subscriber("nlopcontrol_planner/opt", Optimization, optCallback)

    rate = rospy.Rate(30) # 50hz
    plt.ion()
    plt.show()

    while not rospy.is_shutdown():
        data_plotter.draw_frame()
        rate.sleep()
        

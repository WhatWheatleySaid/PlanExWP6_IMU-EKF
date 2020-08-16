#!/usr/bin/python
import rospy
from std_srvs.srv import Empty, EmptyResponse
from gazebo_msgs.srv import SetModelConfiguration, SetModelConfigurationRequest, SetModelConfigurationResponse, SetModelState
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import Imu as ImuMsg
from geometry_msgs.msg import Vector3Stamped as MagneticMessage
import time
import Tkinter as tk
import tkMessageBox as messagebox
import tkFileDialog
import csv

import threading

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib.figure import Figure
import numpy as np

from kalman.ekf import ekf_ori_estimation as ekf
import kalman.quaternion_tools as qtools

class ControllerGUI(tk.Frame):
    def __init__(self,controller_node, *args, **kwargs):
        tk.Frame.__init__(self, *args, **kwargs)
        self.canvas_x = 14
        self.canvas_y = 5
        controller_node.GUI = self
        controller_node._pause_physics_client()

        self.plot_frame = tk.Frame(master = self)
        #matplotlib canvas:
        self.handles = []
        self.mpl_frame = tk.Frame(master = self.plot_frame)
        self.fig = Figure(figsize=(self.canvas_x, self.canvas_y), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title('live data from /imu and /magnectic - topics')
        self.ax.set_xlabel('t [s]')
        self.canvas = FigureCanvasTkAgg(self.fig, master = self.mpl_frame)
        self.canvas.draw()
        self.toolbar = NavigationToolbar2TkAgg(self.canvas, self.mpl_frame)
        self.toolbar.update()
        # self.mpl_frame.pack(side = tk.TOP, fill = tk.BOTH, expand = 1)
        self.mpl_frame.grid(row = 0, column = 0)

        #matplotlib 3d canvas:
        self.handles_3d = []
        self.mpl_3d_frame = tk.Frame(master = self.plot_frame)
        self.fig_3d = Figure(figsize=(self.canvas_y, self.canvas_y), dpi=100)
        self.canvas_3d = FigureCanvasTkAgg(self.fig_3d, master = self.mpl_3d_frame)
        self.canvas_3d.draw()
        self.ax_3d = self.fig_3d.add_subplot(111, projection = '3d')
        self.ax_3d.set_title('estimated position and orientation by EKF')
        self.ax_3d.set_xlabel('global x')
        self.ax_3d.set_ylabel('global y')
        self.ax_3d.set_zlabel('global z')
        self.ax_3d.set_xlim([-5,5])
        self.ax_3d.set_ylim([-5,5])
        self.ax_3d.set_zlim([0,5])
        self.mpl_3d_frame.grid(row = 0, column = 1)
        self.x_axis = np.array([1,0,0])*3
        self.y_axis = np.array([0,1,0])*3
        self.z_axis = np.array([0,0,1])*3


        self.plot_frame.pack(side = tk.TOP, fill = tk.BOTH, expand = 1)
        #setting up checkboxes for the plot
        self.checkbox_frame = tk.Frame(master=self.mpl_frame)
        self.checkbox_var_list = []
        number_of_plots = 8
        plot_names = [  'angular-vel-x',
                        'angular-vel-y',
                        'angular-vel-x',
                        'linear-acc-x',
                        'linear-acc-y',
                        'linear-acc-z',
                        'mag-x',
                        'mag-y',
                        'mag-z']
        plot_data = [
            [[0], [0], 'green', 'angular-vel-x [1/s]'],
            [[0], [0], 'blue', 'angular-vel-y [1/s]'],
            [[0], [0], 'cyan', 'angular-vel-x [1/s]'],
            [[0], [0], 'black', 'linear-acc-x [m/s^2]'],
            [[0], [0], 'red', 'linear-acc-y [m/s^2]'],
            [[0], [0], 'orange', 'linear-acc-z [m/s^2]'],
            [[0], [0], 'magenta', 'mag-x'],
            [[0], [0], 'orchid', 'mag-y'],
            [[0], [0], 'darkorchid', 'mag-z']
        ]

        for pd in plot_data:
                self.handles.append(self.ax.plot(pd[0],pd[1],color = pd[2], linestyle = '-', label = pd[3]))

        for i, name in zip(range(0,number_of_plots+1), plot_names):
            self.checkbox_var_list.append(tk.IntVar())
            self.checkbox_var_list[i].set(1)
            tk.Checkbutton(self.checkbox_frame, text = name, variable = self.checkbox_var_list[i], command = self._checkbox_callback).pack(side = tk.TOP, fill = tk.BOTH, expand = 1)
        self.checkbox_frame.pack(side = tk.RIGHT, fill = tk.Y)


        self.controller_node = controller_node
        self.pause_button = tk.Button(master = self, text='pause', command = self.controller_node._pause_physics_client)
        self.unpause_button = tk.Button(master = self, text='unpause', command = self.controller_node._unpause_physics_client)
        self.reset_button = tk.Button(master = self, text='reset', command = self.reset_cube)
        self.save_button = tk.Button(master = self, text='save to .CSV', command = self.save_data)
    
        #tkinter variables:
        self.x_pos_var = tk.StringVar()
        self.x_pos_var.set('0')
        self.y_pos_var = tk.StringVar()
        self.y_pos_var.set('0')
        self.z_pos_var = tk.StringVar()
        self.z_pos_var.set('10')
        self.x_ori_var = tk.StringVar()
        self.x_ori_var.set('0')
        self.y_ori_var = tk.StringVar()
        self.y_ori_var.set('0')
        self.z_ori_var = tk.StringVar()
        self.z_ori_var.set('0')
        self.w_ori_var = tk.StringVar()
        self.w_ori_var.set('0')
        
        self.x_ang_var = tk.StringVar()
        self.x_ang_var.set('0')
        self.y_ang_var = tk.StringVar()
        self.y_ang_var.set('0')
        self.z_ang_var = tk.StringVar()
        self.z_ang_var.set('0')

        self.x_linv_var = tk.StringVar()
        self.x_linv_var.set('0')
        self.y_linv_var = tk.StringVar()
        self.y_linv_var.set('0')
        self.z_linv_var = tk.StringVar()
        self.z_linv_var.set('0')

        #position frame and entries/labels:
        self.position_frame = tk.LabelFrame(master = self, text = 'position')
        tk.Label(master = self.position_frame, text = '\tx: ').grid(column = 0, row = 0)
        tk.Label(master = self.position_frame, text = '\ty: ').grid(column = 2, row = 0)
        tk.Label(master = self.position_frame, text = '\tz: ').grid(column = 4, row = 0)
        self.x_pos_entry = tk.Entry(self.position_frame, textvariable = self.x_pos_var)
        self.y_pos_entry = tk.Entry(self.position_frame, textvariable = self.y_pos_var)
        self.z_pos_entry = tk.Entry(self.position_frame, textvariable = self.z_pos_var)
        self.x_pos_entry.grid(column = 1, row = 0)
        self.y_pos_entry.grid(column = 3, row = 0)
        self.z_pos_entry.grid(column = 5, row = 0)
        self.position_frame.pack(fill = tk.BOTH, side = tk.BOTTOM)

        #angular velocity frame and entries/labels:
        self.angularv_frame = tk.LabelFrame(master = self, text = 'angular velocity (1/s)')
        tk.Label(master = self.angularv_frame, text = '\tx: ').grid(column = 0, row = 0)
        tk.Label(master = self.angularv_frame, text = '\ty: ').grid(column = 2, row = 0)
        tk.Label(master = self.angularv_frame, text = '\tz: ').grid(column = 4, row = 0)
        self.x_ang_entry = tk.Entry(self.angularv_frame, textvariable = self.x_ang_var)
        self.y_ang_entry = tk.Entry(self.angularv_frame, textvariable = self.y_ang_var)
        self.z_ang_entry = tk.Entry(self.angularv_frame, textvariable = self.z_ang_var)
        self.x_ang_entry.grid(column = 1, row = 0)
        self.y_ang_entry.grid(column = 3, row = 0)
        self.z_ang_entry.grid(column = 5, row = 0)
        self.angularv_frame.pack(fill = tk.BOTH, side = tk.BOTTOM)

        #linear velocity frame and entries/labels
        self.linearv_frame = tk.LabelFrame(master = self, text = 'linear velocity (m/s)')
        tk.Label(master = self.linearv_frame, text = '\tx: ').grid(column = 0, row = 0)
        tk.Label(master = self.linearv_frame, text = '\ty: ').grid(column = 2, row = 0)
        tk.Label(master = self.linearv_frame, text = '\tz: ').grid(column = 4, row = 0)
        self.x_linv_entry = tk.Entry(self.linearv_frame, textvariable = self.x_linv_var)
        self.y_linv_entry = tk.Entry(self.linearv_frame, textvariable = self.y_linv_var)
        self.z_linv_entry = tk.Entry(self.linearv_frame, textvariable = self.z_linv_var)
        self.x_linv_entry.grid(column = 1, row = 0)
        self.y_linv_entry.grid(column = 3, row = 0)
        self.z_linv_entry.grid(column = 5, row = 0)
        self.linearv_frame.pack(fill = tk.BOTH, side = tk.BOTTOM)

        #orientatin frame and entries/labels:
        self.orientation_frame = tk.LabelFrame(master = self, text = 'orientation (quaternion)')
        tk.Label(master = self.orientation_frame, text = '\tx: ').grid(column = 0, row = 0)
        tk.Label(master = self.orientation_frame, text = '\ty: ').grid(column = 2, row = 0)
        tk.Label(master = self.orientation_frame, text = '\tz: ').grid(column = 4, row = 0)
        tk.Label(master = self.orientation_frame, text = '\tw: ').grid(column = 6, row = 0)
        self.x_ori_entry = tk.Entry(self.orientation_frame, textvariable = self.x_ori_var)
        self.y_ori_entry = tk.Entry(self.orientation_frame, textvariable = self.y_ori_var)
        self.z_ori_entry = tk.Entry(self.orientation_frame, textvariable = self.z_ori_var)
        self.w_ori_entry = tk.Entry(self.orientation_frame, textvariable = self.w_ori_var)
        self.x_ori_entry.grid(column = 1, row = 0)
        self.y_ori_entry.grid(column = 3, row = 0)
        self.z_ori_entry.grid(column = 5, row = 0)
        self.w_ori_entry.grid(column = 7, row = 0)
        self.orientation_frame.pack(fill = tk.BOTH, side = tk.BOTTOM)

        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
        self.canvas_3d.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
        self.pause_button.pack(fill = tk.BOTH, side = tk.BOTTOM)
        self.unpause_button.pack(fill = tk.BOTH, side = tk.BOTTOM)
        self.reset_button.pack(fill = tk.BOTH, side = tk.BOTTOM)
        self.save_button.pack(fill = tk.BOTH, side = tk.BOTTOM)
        self.pack(fill = tk.BOTH)
        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)
        controller_node._reset_cube()
        plot_data = [
            [self.x_axis, 'red', 'x'],
            [self.y_axis, 'green', 'y'],
            [self.z_axis, 'blue', 'z'],
        ]
        for pd in plot_data:
            self.handles_3d.append( self.ax_3d.plot( [0,pd[0][0]], [0,pd[0][1]], [0,pd[0][2]], color = pd[1]) )

    def on_closing(self):
        rospy.signal_shutdown('GUI was closed, shutting down simulationcontroller node')
        self.master.destroy()

    def _checkbox_callback(self):
        if self.controller_node.data_list_imu and self.controller_node.simulation_paused:
            self.plot_data(self.controller_node.data_list_imu, self.controller_node.data_list_mag)
            
    def reset_cube(self):
        self._set_cube_state()
        self.controller_node._reset_cube()

    def _set_cube_state(self):
        try:
            self.controller_node.modelstate.pose.position.x = float(self.x_pos_var.get())
            self.controller_node.modelstate.pose.position.y = float(self.y_pos_var.get())
            self.controller_node.modelstate.pose.position.z = float(self.z_pos_var.get())
            self.controller_node.modelstate.pose.orientation.x = float(self.x_ori_var.get())
            self.controller_node.modelstate.pose.orientation.y = float(self.y_ori_var.get())
            self.controller_node.modelstate.pose.orientation.z = float(self.z_ori_var.get())
            self.controller_node.modelstate.pose.orientation.w = float(self.w_ori_var.get())
            self.controller_node.modelstate.twist.angular.x = float(self.x_ang_var.get())
            self.controller_node.modelstate.twist.angular.y = float(self.y_ang_var.get())
            self.controller_node.modelstate.twist.angular.z = float(self.z_ang_var.get())
            self.controller_node.modelstate.twist.linear.x = float(self.x_linv_var.get())
            self.controller_node.modelstate.twist.linear.y = float(self.y_linv_var.get())
            self.controller_node.modelstate.twist.linear.z = float(self.z_linv_var.get())
        except:
            messagebox.showerror('Error', 'Setting cube state failed! Check if your inputs are valid floatingpoint numbers! Setting default values.')
            self.controller_node.modelstate.pose.position.x = 0
            self.controller_node.modelstate.pose.position.y = 0
            self.controller_node.modelstate.pose.position.z = 10
            self.controller_node.modelstate.pose.orientation.x = 0
            self.controller_node.modelstate.pose.orientation.y = 0
            self.controller_node.modelstate.pose.orientation.z = 0
            self.controller_node.modelstate.pose.orientation.w = 0
            self.controller_node.modelstate.twist.angular.x = 0
            self.controller_node.modelstate.twist.angular.y = 0
            self.controller_node.modelstate.twist.angular.z = 0
            self.controller_node.modelstate.twist.linear.x = 0
            self.controller_node.modelstate.twist.linear.y = 0
            self.controller_node.modelstate.twist.linear.z = 0


    def save_data(self):
        dir = tkFileDialog.asksaveasfilename(title = 'select place to save', defaultextension = '.csv')
        if dir != '' and dir != None:
           
            with open(dir, 'w') as csvfile:
                print('saving...')
                csvwriter = csv.writer(csvfile, delimiter= '\t')
                #write header:
                csvwriter.writerow(['sequence', 'time_seconds', 'time_nseconds',
                                    'orientation_x', 'orientation_y','orientation_z', 'orientation_w',
                                    'angular_velocity_x','angular_velocity_y','angular_velocity_z',
                                    'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z',
                                    'mag_x', 'mag_y', 'mag_z'])
                
                for data_imu, data_mag in zip(self.controller_node.data_list_imu, self.controller_node.data_list_mag):
                    seq = data_imu.header.seq
                    ts = data_imu.header.stamp.secs
                    tns = data_imu.header.stamp.nsecs

                    ox = data_imu.orientation.x
                    oy = data_imu.orientation.y
                    oz = data_imu.orientation.z
                    ow = data_imu.orientation.w

                    avx = data_imu.angular_velocity.x
                    avy = data_imu.angular_velocity.y
                    avz = data_imu.angular_velocity.z

                    lax = data_imu.linear_acceleration.x
                    lay = data_imu.linear_acceleration.y
                    laz = data_imu.linear_acceleration.z

                    mag_x = data_mag.vector.x
                    mag_y = data_mag.vector.y
                    mag_z = data_mag.vector.z
                    csvwriter.writerow([seq,ts,tns,ox,oy,oz,ow, avx,avy,avz,lax,lay,laz,mag_x,mag_y,mag_z])
                print('done')

    def plot_data(self,data_list_imu, data_list_mag):
        lax = []
        lay = []
        laz = []
        seq = []
        avx = []
        avy = []
        avz = []
        magx = []
        magy = []
        magz = []
        ts_imu = []
        ts_mag = []
        for data in data_list_imu:
            ts_imu.append(data.header.stamp.secs + data.header.stamp.nsecs*1e-9)
            seq.append(data.header.seq)
            avx.append(data.angular_velocity.x)
            avy.append(data.angular_velocity.y)
            avz.append(data.angular_velocity.z)
            lax.append(data.linear_acceleration.x)
            lay.append(data.linear_acceleration.y)
            laz.append(data.linear_acceleration.z)
        for data in data_list_mag:
            ts_mag.append(data.header.stamp.secs + data.header.stamp.nsecs*1e-9)
            magx.append(data.vector.x)
            magy.append(data.vector.y)
            magz.append(data.vector.z)


        plot_data = [
            [ts_imu, avx, 'green', 'angular-vel-x [1/s]'],
            [ts_imu, avy, 'blue', 'angular-vel-y [1/s]'],
            [ts_imu, avz, 'cyan', 'angular-vel-x [1/s]'],
            [ts_imu, lax, 'black', 'linear-acc-x [m/s^2]'],
            [ts_imu, lay, 'red', 'linear-acc-y [m/s^2]'],
            [ts_imu, laz, 'orange', 'linear-acc-z [m/s^2]'],
            [ts_mag, magx, 'magenta', 'mag-x'],
            [ts_mag, magy, 'orchid', 'mag-y'],
            [ts_mag, magx, 'darkorchid', 'mag-z']
        ]

        for pd, var, handle in zip(plot_data, self.checkbox_var_list, self.handles):
            if var.get():
                handle[0].visible = True
                handle[0].set_xdata(pd[0])
                handle[0].set_ydata(pd[1])
            else:
                handle[0].visible = False

        self.ax.legend(bbox_to_anchor=(1.04,1), loc="upper left")
        self.ax.set_xlim(xmin = ts_imu[0], xmax = ts_imu[-1])
        self.fig.subplots_adjust(right=0.8, left = 0.05, top = 0.95, bottom = .15)
        self.ax.set_ylim(ymin = -20, ymax = 20)
        self.canvas.draw()

        #EKF filtered orientation/pos plot:
        eulers = qtools.quat2euler(self.controller_node.quat_pre)
        x_axis = self.get_rotated_axis(eulers, self.x_axis)
        y_axis = self.get_rotated_axis(eulers, self.y_axis)
        z_axis = self.get_rotated_axis(eulers, self.z_axis)
        axis_list = [x_axis, y_axis, z_axis]
        # plot_data = [
        #     [x_axis, 'red', 'x'],
        #     [y_axis, 'green', 'y'],
        #     [z_axis, 'blue', 'z'],
        # ]
        # if self.handles_3d:
        #     for handle in self.handles_3d:
        #         handle[0].remove()
        # self.handles_3d = []
        # for pd in plot_data:
        #     self.handles_3d.append( self.ax_3d.plot( [0,pd[0][0]], [0,pd[0][1]], [0,pd[0][2]], color = pd[1]) )
        for handle,axis in zip(self.handles_3d, axis_list):
            handle[0].set_xdata([0, axis[0]])
            handle[0].set_ydata([0, axis[1]])
            handle[0].set_3d_properties([0, axis[2]])
        self.canvas_3d.draw()
        return

    def get_rotated_axis(self, eulers, axis):
        axis = np.matmul(self.rot_x(eulers[0]), axis)
        axis = np.matmul(self.rot_y(eulers[1]), axis)
        axis = np.matmul(self.rot_z(eulers[2]), axis)
        return axis

    def rot_x(self,phi):
        '''returns rotational matrix around x, phi in rad'''
        return np.array([[1,0,0],[0,np.cos(phi),-np.sin(phi)],[0,np.sin(phi),np.cos(phi)]])

    def rot_y(self, beta):
        '''returns rotational matrix around y, beta in rad'''
        return np.array([ [np.cos(beta), 0 , np.sin(beta)] , [0,1,0] , [-np.sin(beta), 0, np.cos(beta)] ])

    def rot_z(self,rho):
        '''returns rotational matrix around z, rho in rad'''
        return np.array([[np.cos(rho),-np.sin(rho),0],[np.sin(rho),np.cos(rho),0],[0,0,1]])  

class SimulationController(object):
    #Class for simulation control
    def __init__(self, sensor_rate = 50, GUI = None, name='simulationcontroller', objectname='unit_box'):
        self.name = name
        self.GUI = GUI
        self.objectname = objectname
        self.simulation_paused = True
        self.quat_pre = [1,0,0,0]
        self.data_list_imu = []
        self.data_list_mag = []
        self.sensor_rate = sensor_rate
        self.modelstate = ModelState()
        self.modelstate.model_name = 'simple_cube'
        self.modelstate.pose.position.x = 0
        self.modelstate.pose.position.y = 0
        self.modelstate.pose.position.z = 10
        self.modelstate.pose.orientation.x = 0
        self.modelstate.pose.orientation.y = 0
        self.modelstate.pose.orientation.z = 0
        self.modelstate.pose.orientation.w = 0

        self.resting_modelstate = ModelState()
        self.resting_modelstate.model_name = 'simple_cube'
        self.resting_modelstate.pose.position.x = 0
        self.resting_modelstate.pose.position.y = 0
        self.resting_modelstate.pose.position.z = 0
        self.resting_modelstate.pose.orientation.x = 0
        self.resting_modelstate.pose.orientation.y = 0
        self.resting_modelstate.pose.orientation.z = 0
        self.resting_modelstate.pose.orientation.w = 0
        rospy.init_node(self.name)

        self.reset_flag = True
        #wait for services to be available
        rospy.loginfo('waiting for /gazebo/pause_physics service... (start gazebo via "rosrun gazebo_ros gazebo")')
        rospy.wait_for_service('/gazebo/pause_physics')
        rospy.loginfo('The service \'/gazebo/pause_physics\' has been found')

        rospy.loginfo('waiting for /gazebo/set_model_state service... (start gazebo via "rosrun gazebo_ros gazebo")')
        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.loginfo('The service \'/gazebo/set_model_state\' has been found')

        self.pause_physics_client = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_physics_client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.set_cube_state_client = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.sub_imu = rospy.Subscriber('/imu', ImuMsg, callback=self._imu_topic_callback)
        self.sub_mag = rospy.Subscriber('/magnetic', MagneticMessage, callback=self._magnetic_topic_callback)
        self.plot_rate = rospy.Rate(5)
        self.update_plot_thread = threading.Thread(target = self.update_plot)
        self.update_plot_thread.daemon = True
        self.update_plot_thread.start()

    def update_plot(self):
        while not rospy.is_shutdown():
            self.plot_rate.sleep()
            if not rospy.is_shutdown() and self.data_list_imu:
                self.GUI.plot_data(self.data_list_imu, self.data_list_mag)
            
        
    def _imu_topic_callback(self,data):
        self.data_list_imu.append(data)
        self.calculate_current_quat()

    def calculate_current_quat(self):
        if self.data_list_imu and self.data_list_mag:
            if len(self.data_list_imu) > 1:
                gyr_pre = self.data_list_imu[-2].angular_velocity
                gyr_pre = np.array([gyr_pre.x, gyr_pre.y, gyr_pre.z])
                quat_pre = self.quat_pre
            else:
                gyr_pre = np.array([0 ,0, 0])
                quat_pre = np.array([0,0,0,1])
            acc = self.data_list_imu[-1].linear_acceleration
            mag = self.data_list_mag[-1].vector
            acc = [acc.x, acc.y, acc.z]
            mag = [mag.x, mag.y, mag.z]
            self.quat_pre = ekf(self.sensor_rate,  gyr_pre, quat_pre, acc, mag)

    def _magnetic_topic_callback(self,data):
        self.data_list_mag.append(data)

    def _pause_physics_client(self):
        self.pause_physics_client()
        self.simulation_paused = True
        rospy.loginfo('Pause request has been sent')
    
    def _unpause_physics_client(self):
        if self.reset_flag:
            self.reset_flag = False
            self.set_cube_state_client(self.resting_modelstate)
            self.unpause_physics_client()
            self.GUI.master.after(1500, self._set_state)
        self.simulation_paused = False
        self.unpause_physics_client()
        rospy.loginfo('Unpause request has been sent')

    def _set_resting_state(self):
        self.set_cube_state_client(self.resting_modelstate)

    def _set_state(self):
        self.set_cube_state_client(self.modelstate)

    def _reset_cube(self):
        '''
        resets the cubes position and empties gathered data list
        '''
        self.pause_physics_client()
        self.data_list_imu = []
        self.set_cube_state_client(self.modelstate)
        self.reset_flag = True
        rospy.loginfo('reset request has been sent')


if __name__ == '__main__':
    
    root = tk.Tk()
    root.title('Simulation Controller')
    controller_node = SimulationController()
    gui = ControllerGUI(controller_node,master = root)
    root.resizable(False,False)
    root.mainloop()
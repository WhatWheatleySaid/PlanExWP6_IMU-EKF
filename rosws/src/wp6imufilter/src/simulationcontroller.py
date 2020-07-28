#!/bin/python
import rospy
from std_srvs.srv import Empty, EmptyResponse
from gazebo_msgs.srv import SetModelConfiguration, SetModelConfigurationRequest, SetModelConfigurationResponse, SetModelState
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import Imu as ImuMsg
import time
import Tkinter as tk
import tkFileDialog
import csv

import threading

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib.figure import Figure

class ControllerGUI(tk.Frame):
    def __init__(self,controller_node, *args, **kwargs):
        tk.Frame.__init__(self, *args, **kwargs)
        controller_node.GUI = self
        controller_node._pause_physics_client()
        
        #matplotlib canvas:
        self.handle = None
        self.mpl_frame = tk.Frame(master = self)
        self.fig = Figure(figsize=(5, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.plot([0,0], [0,0])
        self.canvas = FigureCanvasTkAgg(self.fig, master = self.mpl_frame)
        self.canvas.draw()
        self.toolbar = NavigationToolbar2TkAgg(self.canvas, self.mpl_frame)
        self.toolbar.update()
        self.mpl_frame.pack(side = tk.TOP, fill = tk.BOTH, expand = 1)



        self.controller_node = controller_node
        self.pause_button = tk.Button(master = self, text='pause', command = self.controller_node._pause_physics_client)
        self.unpause_button = tk.Button(master = self, text='unpause', command = self.controller_node._unpause_physics_client)
        self.reset_button = tk.Button(master = self, text='reset', command = self.controller_node._reset_cube)
        self.save_button = tk.Button(master = self, text='save to .CSV', command = self.save_data)
        
        # self.toolbar.pack(side=tkinter.TOP, fill=tkinter.BOTH, expand=1)

        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
        self.pause_button.pack(fill = tk.BOTH, side = tk.BOTTOM)
        self.unpause_button.pack(fill = tk.BOTH, side = tk.BOTTOM)
        self.reset_button.pack(fill = tk.BOTH, side = tk.BOTTOM)
        self.save_button.pack(fill = tk.BOTH, side = tk.BOTTOM)
        self.pack(fill = tk.BOTH)
        controller_node._reset_cube()
    
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
                                    'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z'])
                
                for data in self.controller_node.data_list:
                    seq = data.header.seq
                    ts = data.header.stamp.secs
                    tns = data.header.stamp.nsecs

                    ox = data.orientation.x
                    oy = data.orientation.y
                    oz = data.orientation.z
                    ow = data.orientation.w

                    avx = data.angular_velocity.x
                    avy = data.angular_velocity.y
                    avz = data.angular_velocity.z

                    lax = data.linear_acceleration.x
                    lay = data.linear_acceleration.y
                    laz = data.linear_acceleration.z
                    csvwriter.writerow([seq,ts,tns,ox,oy,oz,ow, avx,avy,avz,lax,lay,laz])
                print('done')

    def plot_data(self,data_list):
        laz = []
        ts = []
        for data in data_list:
            ts.append(data.header.stamp.secs)
            # avx = data.angular_velocity.x
            # avy = data.angular_velocity.y
            # avz = data.angular_velocity.z
            # lax = data.linear_acceleration.x
            # lay = data.linear_acceleration.y
            laz.append(data.linear_acceleration.z)
        if self.handle != None:
            self.handle.remove()
        [self.handle] = self.ax.plot(ts, laz, linestyle = '-', color = 'b', marker = '.')
        self.ax.relim()
        self.ax.autoscale_view()
        self.ax.set_xlim(xmin = ts[0], xmax = ts[-1])
        self.canvas.draw()
        return
        
class SimulationController(object):
    #Class for simulation control
    def __init__(self, GUI = None, name='simulationcontroller', objectname='unit_box'):
        self.name = name
        self.GUI = GUI
        self.objectname = objectname
        self.data_list = []
        self.modelstate = ModelState()
        self.modelstate.model_name = 'simple_cube'
        self.modelstate.pose.position.x = 0
        self.modelstate.pose.position.y = 0
        self.modelstate.pose.position.z = 5
        self.modelstate.pose.orientation.x = 0
        self.modelstate.pose.orientation.y = 0
        self.modelstate.pose.orientation.z = 0
        self.modelstate.pose.orientation.w = 0
        rospy.init_node(self.name)

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
        self.sub = rospy.Subscriber('/imu', ImuMsg, callback=self._imu_topic_callback)
        
        self.plot_rate = rospy.Rate(0.5)
        self.update_plot_thread = threading.Thread(target = self.update_plot)
        self.update_plot_thread.daemon = True
        self.update_plot_thread.start()

    def update_plot(self):
        while not rospy.is_shutdown():
            self.plot_rate.sleep()
            self.GUI.plot_data(self.data_list)
        
    def _imu_topic_callback(self,data):
        rospy.loginfo('Received IMU msg: {0}'.format(data))
        self.data_list.append(data)
        return

    def _pause_physics_client(self):
        self.pause_physics_client()
        rospy.loginfo('Pause request has been sent')
    
    def _unpause_physics_client(self):
        self.unpause_physics_client()
        rospy.loginfo('Unpause request has been sent')

    def _reset_cube(self):
        '''
        resets the cubes position and empties gathered data list
        '''
        self.data_list = []
        self.set_cube_state_client(self.modelstate)
        rospy.loginfo('reset request has been sent')


if __name__ == '__main__':
    
    root = tk.Tk()
    controller_node = SimulationController()
    gui = ControllerGUI(controller_node,master = root)
    root.mainloop()
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

class ControllerGUI(tk.Frame):
    def __init__(self,controller_node, *args, **kwargs):
        tk.Frame.__init__(self, *args, **kwargs)
        self.controller_node = controller_node

        self.pause_button = tk.Button(text='pause', command = self.controller_node._pause_physics_client)
        self.unpause_button = tk.Button(text='unpause', command = self.controller_node._unpause_physics_client)
        self.reset_button = tk.Button(text='reset', command = self.controller_node._reset_cube)
        self.save_button = tk.Button(text='save to .CSV', command = self.save_data)
        self.pause_button.pack(fill = tk.BOTH)
        self.unpause_button.pack(fill = tk.BOTH)
        self.reset_button.pack(fill = tk.BOTH)
        self.save_button.pack(fill = tk.BOTH)
    
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


class SimulationController(object):
    #Class for simulation control
    def __init__(self, name='simulationcontroller', objectname='unit_box'):
        self.name = name
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

if __name__ == '__main__':
    
    root = tk.Tk()
    controller_node = SimulationController()
    gui = ControllerGUI(controller_node,master = root)
    root.mainloop()
#!/bin/python
import rospy
from std_srvs.srv import Empty, EmptyResponse
from gazebo_msgs.srv import SetModelConfiguration, SetModelConfigurationRequest, SetModelConfigurationResponse
import time
import Tkinter as tk

class ControllerGUI(tk.Frame):
    def __init__(self,controller_node, *args, **kwargs):
        tk.Frame.__init__(self, *args, **kwargs)
        self.controller_node = controller_node
        self.pause_button = tk.Button(text='pause', command = controller_node._pause_physics_client)
        self.unpause_button = tk.Button(text='unpause', command = controller_node._unpause_physics_client)
        self.pause_button.pack()
        self.unpause_button.pack()


class SimulationController(object):
    #Class for simulation control
    def __init__(self, name='simulationcontroller', objectname='unit_box'):
        self.name = name
        self.objectname = objectname

        rospy.init_node(self.name)

        #wait for service to be available
        rospy.wait_for_service('/gazebo/pause_physics')
        rospy.loginfo('The service \'/gazebo/pause_physics\' has been found')

        self.pause_physics_client = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_physics_client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

    
    # def run(self):
    #     rospy.loginfo('Starting simulation controller')
    #     while not rospy.is_shutdown():
    #         self._unpause_physics_client()
    #         self._pause_physics_client()

    def _pause_physics_client(self):
        self.pause_physics_client()
        rospy.loginfo('Pause request has been sent')
        # time.sleep(4)
    
    def _unpause_physics_client(self):
        self.unpause_physics_client()
        rospy.loginfo('Unpause request has been sent')
        # time.sleep(4)

if __name__ == '__main__':
    
    root = tk.Tk()
    controller_node = SimulationController()
    gui = ControllerGUI(controller_node,master = root)
    root.mainloop()
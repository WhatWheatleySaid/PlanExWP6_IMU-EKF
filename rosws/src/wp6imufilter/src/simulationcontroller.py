#!/bin/python
import rospy
from std_srvs.srv import Empty, EmptyResponse
from gazebo_msgs.srv import SetModelConfiguration, SetModelConfigurationRequest, SetModelConfigurationResponse

class SimulationController(object):
    #Class for simulation control
    def __init__(self, name='simulationcontroller', objectname='unit_box'):
        self.name = name
        self.objectname = objectname

        rospy.init_node(self.name)

        self.working_freq = rospy.Rate(0.25)
        
        #wait for service to be available
        rospy.wait_for_service('/gazebo/pause_physics')
        rospy.loginfo('The service \'/gazebo/pause_physics\' has been found')

        self.pause_physics_client = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_physics_client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    
    def run(self):
        rospy.loginfo('Starting simulation controller')
        while not rospy.is_shutdown():
            self.unpause_physics_client()
            rospy.loginfo('Unpause request has been sent')
            self.working_freq.sleep()
            self.pause_physics_client()
            rospy.loginfo('Pause request has been sent')
            self.working_freq.sleep()

if __name__ == '__main__':
    simulationcontroller = SimulationController()
    simulationcontroller.run()
#!/usr/bin/python

import rospy
import actionlib

from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi
from Queue import Queue
import math





class MoveBaseInterface(object):
    """
    A Simple Inteface for move base. For testing, run this, this script will generate random goals on a 5 meter square
    """
    def __init__(self):
        
        # Create interface to move_base
        self.move_base=actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('waiting for move_base to come online')
        self.move_base.wait_for_server(rospy.Duration(10))
        
        self.nav_goals = Queue()

        # Creating a simple publisher to publish on cmd_vel for manual navigation
        self.cmd_vel_topic = rospy.get_param('cmd_vel_topic', default='cmd_vel')
        self.cmd_vel = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)


        # Creating a publisher for visualizing goals in rviz
        self.visualizer = rospy.Publisher('goal_marks', Marker, queue_size=10)
        self.marker=Marker()
        self.marker.ns = 'waypoints' 
        self.marker.id = 0
        self.marker.type = Marker.CUBE_LIST
        self.marker.action = Marker.ADD
        self.marker.lifetime = rospy.Duration(0) #forever
        self.marker.scale.x = 0.2
        self.marker.scale.y = 0.2
        self.marker.scale.z = 0.1
        self.marker.color.r = 1.0
        self.marker.color.g = 0.7
        self.marker.color.b = 1.0
        self.marker.color.a = 1.0
        self.marker.header.frame_id = 'map'
        self.marker.points = list()


        # Important to cancel goals in case of shutdown
        rospy.on_shutdown(self.poweroff)
        

    def create_2d_goal(self, x, y, theta):
        # Theta is assumed to be in degress!
        point = Point()
        point.x = x
        point.y = y
        angle = math.pi*theta / 180.0
        quat_angle = quaternion_from_euler(0,0,angle, axes='sxyz')
        quaternion = Quaternion(*quat_angle)
        pose = Pose(point, quaternion)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose = pose
        goal.target_pose.header.stamp = rospy.Time.now()
        self.nav_goals.put(goal)

    def send_goal (self):
        if not self.nav_goals.empty():
            goal = self.nav_goals.get()
        else:
            return
        goal.target_pose.header.stamp = rospy.Time.now()
        self.visualize_goal(goal)
        self.move_base.send_goal(goal)
        finished = self.move_base.wait_for_result(rospy.Duration(30))
        if not finished:
            self.move_base.cancel_goal()
            rospy.loginfo('This took too long')
        else:
            if GoalStatus.SUCCEEDED == self.move_base.get_state():
                rospy.loginfo('Reached!')

    def visualize_goal(self, goal):
        self.marker.header.stamp = rospy.Time.now()
        self.marker.pose.orientation.w=1.0
        p = Point()
        p = goal.target_pose.pose.position
        self.marker.points=[p]
        for i in range(10):
            self.visualizer.publish(self.marker)
    
    def poweroff (self):
        rospy.loginfo("Powering off and cancelling all goals")
        self.move_base.cancel_all_goals()
        rospy.sleep(2)
        # Make sure the rover stops
        self.cmd_vel.publish(Twist())
        rospy.sleep(2)

if __name__ == '__main__':
    rospy.init_node('MoveBaseInterface')
    import random
    move_base = MoveBaseInterface()
    while not rospy.is_shutdown():
        move_base.create_2d_goal(5*random.random(), 5*random.random(), 0.0)
        move_base.send_goal()
        rospy.sleep(1)
    
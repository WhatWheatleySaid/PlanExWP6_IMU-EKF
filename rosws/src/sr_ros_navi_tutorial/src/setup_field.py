#!/usr/bin/python

import rospy
import random
import math
import roslaunch
from turtlesim.srv import Spawn, SpawnRequest
import time
class TortugaConjurer(object):
    def __init__(self):
        rospy.wait_for_service('spawn')
        self.conjuration = rospy.ServiceProxy('spawn', Spawn)
        self.spawned_turtles = []
        self.map_size=rospy.get_param("map_size", default=6.0)
        self.tf_processes = []
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()
        
        
    def conjure_turtles(self, x=None, y=None, theta=None, name=None):
        spell = SpawnRequest()
        if (x,y,theta,name) == (None, None, None, None):
            spell.x = random.random()*self.map_size
            spell.y = random.random()*self.map_size
            spell.theta = random.random()*2*math.pi
            spell.name="turtle"+str(len(self.spawned_turtles)+1)      
        else:
            spell.x = x
            spell.y = y
            spell.theta = theta
            spell.name = name
        node=roslaunch.core.Node('turtle_tf', 'turtle_tf_broadcaster', args=str(spell.name))
        node.name=spell.name+"tf_broadcast"
        node.remap_args.append(('tf', 'tf_old'))
        
        self.tf_processes.append(self.launch.launch(node))
        self.conjuration(spell)
        self.spawned_turtles.append(spell.name)

    #def prepare_field(self)



if __name__ == "__main__":
    rospy.init_node('conjuring_tests')
    rate=rospy.Rate(0.5)
    conj = TortugaConjurer()

    while not rospy.is_shutdown():
        rate.sleep()
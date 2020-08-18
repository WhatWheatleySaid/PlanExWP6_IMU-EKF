#!/usr/bin/env python
# based on:
# PointCloud2 color cube
# https://answers.ros.org/question/289576/understanding-the-bytes-in-a-pcl2-message/
# adapted by: Lennart Kryza for creating a service to generate simply obstacles.

import rospy
import struct

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from sr_ros_navi_tutorial.srv import CreateObstacle, CreateObstacleResponse

class ObstacleMaker(object):
    def __init__(self):
        rospy.init_node("ObstacleMaker")
        self.pub = rospy.Publisher("rtabmap/cloud_obstacles", PointCloud2, queue_size=2)
        self.obstacle_service = rospy.Service('create_obstacle', CreateObstacle, self.publish_obstacle)
        self.publish_list = []

    def publish_obstacle(self, data):
        pack = {}
        number=data.area
        width=0.1
        points = []
        x_origin = data.x
        y_origin = data.y
        z_origin = 0.0
        for i in range(1,number):
            step=width/number
            step=i*step
            x = x_origin-step
            y = y_origin
            pt = [x, y, 0.0]
            points.append(pt)
            x = x_origin+step
            y = y_origin
            pt = [x, y, 0.0]
            points.append(pt)
            x = x_origin
            y = y_origin-step
            pt = [x, y, 0.0]
            points.append(pt)
            x = x_origin
            y = y_origin+step
            pt = [x, y, 0.0]
            points.append(pt)
        pack['points'] = points
        self.publish_list.append(pack)
        return True
    def run (self):
        while not rospy.is_shutdown():
            fields = [PointField('points', 1, PointField.FLOAT32, 1)]
            points=[]
            for package in self.publish_list:
                points = package['points']
                header = Header()
                header.frame_id = "map"
                pc2 = point_cloud2.create_cloud_xyz32(header, points)
                pc2.header.stamp = rospy.Time.now()
                self.pub.publish(pc2)
            rospy.sleep(1)


if __name__== "__main__":
    test=ObstacleMaker()
    test.run()
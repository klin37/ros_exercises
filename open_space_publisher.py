#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from ros_exercises.msg import OpenSpace
import numpy as np

def callback(data):
    index = np.argmax(data.ranges)
    dist = data.ranges[index]
    angle = -2*np.pi/3+index*np.pi/300
    msg = OpenSpace(angle = angle, distance = dist)
    # topic_dist = rospy.Publisher('open_space/distance', Float32)
    # topic_angle = rospy.Publisher('open_space/angle', Float32)
    pub = rospy.Publisher('open_space', OpenSpace)
    # rospy.loginfo(dist)
    # rospy.loginfo(angle)
    rospy.loginfo(msg)
    # topic_dist.publish(dist)
    # topic_angle.publish(angle)
    pub.publish(msg)

def open_space_publisher():
    rospy.init_node('open_space_publisher')

    rospy.Subscriber('fake_scan', LaserScan, callback)

    rospy.spin()

if __name__ == '__main__':
    open_space_publisher()
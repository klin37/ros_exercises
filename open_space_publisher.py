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
    publisher = rospy.get_param('publisher', 'open_space')
    pub = rospy.Publisher(publisher, OpenSpace, queue_size=50)
    # rospy.loginfo(dist)
    # rospy.loginfo(angle)
    rospy.loginfo(msg)
    # topic_dist.publish(dist)
    # topic_angle.publish(angle)
    pub.publish(msg)

def open_space_publisher():
    rospy.init_node('open_space_publisher')
    subscriber = rospy.get_param('subscriber', 'fake_scan')
    rospy.Subscriber(subscriber, LaserScan, callback)

    rospy.spin()

if __name__ == '__main__':
    open_space_publisher()
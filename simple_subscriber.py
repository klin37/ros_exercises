#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import numpy as np

def callback(data):
    pub = rospy.Publisher('random_float_log', Float32, queue_size=50)
    num = np.log(data.data)
    rospy.loginfo(num)
    pub.publish(num)

def simple_subscriber():
    rospy.init_node('simple_subscriber')

    rospy.Subscriber('my_random_float', Float32, callback)

    rospy.spin()

if __name__ == '__main__':
    simple_subscriber()
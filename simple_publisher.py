#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import random

def simple_publisher():
    pub = rospy.Publisher('my_random_float', Float32, queue_size=50)
    rospy.init_node('simple_publisher')
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        num = Float32(data=random.random()*10)
        rospy.loginfo(num)
        pub.publish(num)
        rate.sleep()

if __name__ == '__main__':
    try:
        simple_publisher()
    except rospy.ROSInterruptException:
        pass
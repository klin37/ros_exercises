#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import numpy as np

def fake_scan_publisher():
    pub = rospy.Publisher('fake_scan', LaserScan, queue_size=50)
    rospy.init_node('fake_scan_publisher')
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        header = Header(stamp = now, frame_id = 'base_link')
        laser = LaserScan(header = header, angle_min = -2*np.pi/3, angle_max = 2*np.pi/3, angle_increment = np.pi/300, 
                          scan_time = 0.05, range_min = 1.0, range_max = 10.0, ranges = np.random.rand(401)*9+1.0
                          )
        rospy.loginfo(laser)
        pub.publish(laser)
        rate.sleep()

if __name__ == '__main__':
    try:
        fake_scan_publisher()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import numpy as np

def fake_scan_publisher():
    topic = rospy.get_param('topic', 'fake_scan')
    pub = rospy.Publisher(topic, LaserScan, queue_size=50)
    rospy.init_node('fake_scan_publisher')
    publish_rate = rospy.get_param('rate', 20)
    rate = rospy.Rate(publish_rate)
    angle_min = rospy.get_param('angle_min', -2*np.pi/3)
    angle_max = rospy.get_param('angle_max', 2*np.pi/3)
    range_min = rospy.get_param('range_min', 1.0)
    range_max = rospy.get_param('range_max', 10.0)
    angle_increment = rospy.get_param('angle_increment', np.pi/300)

    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        ranges = np.random.rand(int((angle_max-angle_min)/angle_increment+1))*(range_max-range_min)+range_min
        header = Header(stamp = now, frame_id = 'base_link')
        laser = LaserScan(header = header, angle_min = angle_min, angle_max = angle_max, angle_increment = angle_increment, 
                          scan_time = 1/float(publish_rate), range_min = range_min, range_max = range_max, ranges = ranges
                          )
        rospy.loginfo(laser)
        pub.publish(laser)
        rate.sleep()

if __name__ == '__main__':
    try:
        fake_scan_publisher()
    except rospy.ROSInterruptException:
        pass
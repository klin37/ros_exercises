#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import tf2_ros
import tf.transformations as transformations
import numpy as np


def cam_broadcasts():
    rospy.init_node('static_tf_cam_publisher')
    br = tf2_ros.StaticTransformBroadcaster()
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    left_to_robot = np.array([[1, 0, 0, -0.05], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    right_to_robot = np.array([[1, 0, 0, 0.05], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    
    left_transform = tf2_ros.TransformStamped()
    left_transform.header.stamp = rospy.Time.now()
    left_transform.header.frame_id = 'base_link_gt'
    left_transform.child_frame_id = 'left_cam'
    left_transform.transform.translation.x = left_to_robot[0, 3]
    left_transform.transform.translation.y = left_to_robot[1, 3]
    left_transform.transform.translation.z = left_to_robot[2, 3]
    left_quaternion = transformations.quaternion_from_matrix(left_to_robot)
    left_transform.transform.rotation.x = left_quaternion[0]
    left_transform.transform.rotation.y = left_quaternion[1]
    left_transform.transform.rotation.z = left_quaternion[2]
    left_transform.transform.rotation.w = left_quaternion[3]


    right_transform = tf2_ros.TransformStamped()
    right_transform.header.stamp = rospy.Time.now()
    right_transform.header.frame_id = 'base_link_gt'
    right_transform.child_frame_id = 'right_cam'
    right_transform.transform.translation.x = right_to_robot[0, 3]
    right_transform.transform.translation.y = right_to_robot[1, 3]
    right_transform.transform.translation.z = right_to_robot[2, 3]
    right_quaternion = transformations.quaternion_from_matrix(right_to_robot)
    right_transform.transform.rotation.x = right_quaternion[0]
    right_transform.transform.rotation.y = right_quaternion[1]
    right_transform.transform.rotation.z = right_quaternion[2]
    right_transform.transform.rotation.w = right_quaternion[3]
    br.sendTransform([right_transform, left_transform])

    rospy.spin()

if __name__ == '__main__':
    cam_broadcasts()
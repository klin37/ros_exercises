#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import tf2_ros
import tf.transformations as transformations
import numpy as np


def cam_broadcasts():
    rospy.init_node('dynamic_tf_cam_publisher')
    br = tf2_ros.TransformBroadcaster()
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(20)
    
    left_to_robot = np.array([[1, 0, 0, -0.05], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    right_to_left = np.array([[1, 0, 0, 0.1], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    while not rospy.is_shutdown():
        
        try:
            robot_transform = tfBuffer.lookup_transform('world', 'base_link_gt', rospy.Time())
        except:
            continue
        translation = np.array([robot_transform.transform.translation.x, robot_transform.transform.translation.y, robot_transform.transform.translation.z])
        rotation = np.array([robot_transform.transform.rotation.x, robot_transform.transform.rotation.y, robot_transform.transform.rotation.z, robot_transform.transform.rotation.w])
        robot_matrix = transformations.concatenate_matrices(transformations.translation_matrix(translation), transformations.quaternion_matrix(rotation))
        
        left_cam = np.dot(robot_matrix, left_to_robot)
        left_transform = tf2_ros.TransformStamped()
        left_transform.header.stamp = rospy.Time.now()
        left_transform.header.frame_id = 'world'
        left_transform.child_frame_id = 'left_cam'
        left_transform.transform.translation.x = left_cam[0, 3]
        left_transform.transform.translation.y = left_cam[1, 3]
        left_transform.transform.translation.z = left_cam[2, 3]
        left_quaternion = transformations.quaternion_from_matrix(left_cam)
        left_transform.transform.rotation.x = left_quaternion[0]
        left_transform.transform.rotation.y = left_quaternion[1]
        left_transform.transform.rotation.z = left_quaternion[2]
        left_transform.transform.rotation.w = left_quaternion[3]
        br.sendTransform(left_transform)

        right_transform = tf2_ros.TransformStamped()
        right_transform.header.stamp = rospy.Time.now()
        right_transform.header.frame_id = 'left_cam'
        right_transform.child_frame_id = 'right_cam'
        right_transform.transform.translation.x = right_to_left[0, 3]
        right_transform.transform.translation.y = right_to_left[1, 3]
        right_transform.transform.translation.z = right_to_left[2, 3]
        right_quaternion = transformations.quaternion_from_matrix(right_to_left)
        right_transform.transform.rotation.x = right_quaternion[0]
        right_transform.transform.rotation.y = right_quaternion[1]
        right_transform.transform.rotation.z = right_quaternion[2]
        right_transform.transform.rotation.w = right_quaternion[3]
        br.sendTransform(right_transform)

        rate.sleep()

if __name__ == '__main__':
    cam_broadcasts()
#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import tf2_ros
import tf.transformations as transformations
import numpy as np


def cam_broadcasts():
    rospy.init_node('base_link_tf_pub')
    br = tf2_ros.TransformBroadcaster()
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(20)
    
    left_to_robot = np.array([[1, 0, 0, -0.05], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    while not rospy.is_shutdown():
        
        try:
            left_to_world = tfBuffer.lookup_transform('world', 'left_cam', rospy.Time())
        except:
            continue
        translation = np.array([left_to_world.transform.translation.x, left_to_world.transform.translation.y, left_to_world.transform.translation.z])
        rotation = np.array([left_to_world.transform.rotation.x, left_to_world.transform.rotation.y, left_to_world.transform.rotation.z, left_to_world.transform.rotation.w])
        world_matrix = transformations.concatenate_matrices(transformations.translation_matrix(translation), transformations.quaternion_matrix(rotation))
        
        robot_to_world = np.dot(np.linalg.inv(left_to_robot), world_matrix)
        robot_transform = tf2_ros.TransformStamped()
        robot_transform.header.stamp = rospy.Time.now()
        robot_transform.header.frame_id = 'world'
        robot_transform.child_frame_id = 'base_link_gt_2'
        robot_transform.transform.translation.x = robot_to_world[0, 3]
        robot_transform.transform.translation.y = robot_to_world[1, 3]
        robot_transform.transform.translation.z = robot_to_world[2, 3]
        robot_quaternion = transformations.quaternion_from_matrix(robot_to_world)
        robot_transform.transform.rotation.x = robot_quaternion[0]
        robot_transform.transform.rotation.y = robot_quaternion[1]
        robot_transform.transform.rotation.z = robot_quaternion[2]
        robot_transform.transform.rotation.w = robot_quaternion[3]
        br.sendTransform(robot_transform)

        rate.sleep()

if __name__ == '__main__':
    cam_broadcasts()
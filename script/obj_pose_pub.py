#!/usr/bin/env python
import roslib

import rospy
import math
import tf
from geometry_msgs.msg import Pose

if __name__ == '__main__':
    rospy.init_node('obj_pose_pub')

    listener = tf.TransformListener()

    obj_pose_pub = rospy.Publisher('/obj_pose', Pose, queue_size=1)

    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/camera', '/tag_4', rospy.Time(0))
            pose_msg = Pose()

            pose_msg.position.x = trans[0]
            pose_msg.position.y = trans[1]
            pose_msg.position.z = trans[2]

            pose_msg.orientation.x = rot[0]
            pose_msg.orientation.y = rot[1]
            pose_msg.orientation.z = rot[2]
            pose_msg.orientation.w = rot[2]
            obj_pose_pub.publish(pose_msg)


        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

#!/usr/bin/env python

import numpy as np

import rospy
import math
import tf
from geometry_msgs.msg import Pose
from average_quaternion import average_quaterion
from copy import deepcopy

if __name__ == '__main__':
    rospy.init_node('obj_pose_pub')

    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()

    obj_pose_pub = rospy.Publisher('/object_pose', Pose, queue_size=1)

    rate = rospy.Rate(30.0)

    prev_trans2 = None
    prev_rot2   = None

    prev_trans1 = None
    prev_rot1   = None

    alpha = 0.9
    while not rospy.is_shutdown():
        # we are using two cameras
        br.sendTransform(
                        (0., 0., -0.055),
                        (0., 0., 0., 1.),
                        rospy.Time.now(),
                        "/object_center1","/object1")
        br.sendTransform(
                        (0., 0., -0.055),
                        (0., 0., 0., 1.),
                        rospy.Time.now(),
                        "/object_center2","/object2")

        try:
            (trans1, rot1) = listener.lookupTransform('/origin1', '/object_center1', rospy.Time(0))
            (trans2, rot2) = listener.lookupTransform('/origin2', '/object_center2', rospy.Time(0))

            trans1 = np.array(trans1)
            rot1   = np.array(rot1)
            trans2 = np.array(trans2)
            rot2   = np.array(rot2)

            if prev_rot1 is None:
                prev_rot1 = deepcopy(rot1)
                prev_rot2 = deepcopy(rot2)
                prev_trans1 = deepcopy(trans1)
                prev_trans2 = deepcopy(trans2)
            else:
                trans1 = alpha * prev_trans1 + (1.0 - alpha) * trans1
                trans2 = alpha * prev_trans2 + (1.0 - alpha) * trans2
                #rot1   = alpha * prev_rot1   + (1.0 - alpha) * rot1
                #rot2   = alpha * prev_rot2   + (1.0 - alpha) * rot2

            # if np.linalg.norm(trans1 - prev_trans1) > 0.1 or np.linalg.norm(rot1 - prev_rot1) > 0.1:
            #     trans1 = prev_trans1
            #     rot1   = prev_rot1
            #
            # if np.linalg.norm(trans2 - prev_trans2) > 0.1 or np.linalg.norm(rot2 - prev_rot2) > 0.1:
            #     trans2 = prev_trans2
            #     rot2   = prev_rot2

            # because I transform the vector afterwards
            prev_rot1 = deepcopy(rot1)
            prev_rot2 = deepcopy(rot2)
            prev_trans1 = deepcopy(trans1)
            prev_trans2 = deepcopy(trans2)

            rot1 = [rot1[3], rot1[0], rot1[1], rot1[2]]
            rot2 = [rot2[3], rot2[0], rot2[1], rot2[2]]

            # Take the average pose + average quaternion
            trans = (np.array(trans1) + np.array(trans2)) / 2.0
            rot   = average_quaterion(np.stack([rot1, rot2]))

            pose_msg = Pose()

            pose_msg.position.x = trans[0]
            pose_msg.position.y = trans[1]
            pose_msg.position.z = trans[2]

            pose_msg.orientation.x = rot[1]
            pose_msg.orientation.y = rot[2]
            pose_msg.orientation.z = rot[3]
            pose_msg.orientation.w = rot[0]
            obj_pose_pub.publish(pose_msg)





        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

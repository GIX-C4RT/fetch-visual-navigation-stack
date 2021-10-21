import tf
from tf.transformations import * 
import rospy
import numpy as np

rospy.init_node('tester')
listener = tf.TransformListener()
rate = rospy.Rate(25.0)
br = tf.TransformBroadcaster()
while not rospy.is_shutdown():
    try:
        (trans1,rot1) = listener.lookupTransform('/map', '/camera_pose_frame', rospy.Time(0))
        (trans2,rot2) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
        # global_trans = [trans1[0] - trans2[0], trans1[1] - trans2[1], 0]
        transform1 = concatenate_matrices(translation_matrix(trans1), quaternion_matrix(rot1))
        transform2 = concatenate_matrices(translation_matrix(trans2), quaternion_matrix(rot2))
        global_trans_matrix = np.matmul(transform1, np.linalg.inv(transform2))
        global_trans_quaternion = quaternion_from_matrix(global_trans_matrix)
        global_trans_translation = translation_from_matrix(global_trans_matrix)
        br.sendTransform(global_trans_translation,
                    global_trans_quaternion,
                    rospy.Time.now(),
                    "odom",
                    "map")
        # print "global: ", global_trans
        # print "mc: ", trans1
        # print "bo: ", trans2
        # br.sendTransform(tuple(global_trans),
        #             (0,0,0,1),
        #             rospy.Time.now(),
        #             "odom",
        #             "map")
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

    rate.sleep()
import tf
import rospy

rospy.init_node('tester2')
listener = tf.TransformListener()
rate = rospy.Rate(25.0)
br = tf.TransformBroadcaster()
while not rospy.is_shutdown():
    try:
        (trans,rot) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
        br.sendTransform(tuple(trans),
                tuple(rot),
                rospy.Time.now(),
                "odom",
                "camera_pose_frame")
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    rate.sleep()
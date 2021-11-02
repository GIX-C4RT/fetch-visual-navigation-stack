import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance
from nav_msgs.msg import Odometry
import roslib
import tf
from tf.transformations import *
import tf2_ros
import tf2_geometry_msgs
from utils import * 
import numpy as np

rospy.init_node('map_odom_correction', anonymous=True)
r = rospy.Rate(25)
br = tf.TransformBroadcaster()
tf_buffer = tf2_ros.Buffer(rospy.Duration(300.0)) #tf buffer length

while not rospy.is_shutdown():
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    transform = tf_buffer.lookup_transform("camera_pose_frame",
                                        "base_link", #source frame
                                        rospy.Time(0),
                                        rospy.Duration(5.0)) #get the tf at first available time

    print transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z
    br.sendTransform((transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z),
                    (transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w),
                    rospy.Time.now(),
                    "odom",
                    "map")

    # br.sendTransform((2, 2, 0),
    #                 (0, 0, 0, 1),
    #                 rospy.Time.now(),
    #                 "odom",
    #                 "map")
    r.sleep()
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

init_pose = None

def reinit_pose(msg):
    global init_pose
    init_pose = msg.pose.pose

rospy.init_node('map_odom_bind', anonymous=True)
rospy.Subscriber("initialpose", PoseWithCovarianceStamped, reinit_pose)
r = rospy.Rate(25)
br = tf.TransformBroadcaster()
tf_buffer = tf2_ros.Buffer(rospy.Duration(300.0)) #tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)

while not rospy.is_shutdown():
    if init_pose:
        transform = tf_buffer.lookup_transform("base_link",
                                       "odom", #source frame
                                       rospy.Time(0),
                                       rospy.Duration(5.0)) #get the tf at first available time

        b2o_matrix = msg_to_se3(transform)
        m2i_matrix = msg_to_se3(init_pose)
        final_matrix = np.matmul(m2i_matrix, b2o_matrix)
        br.sendTransform((final_matrix[0][3], final_matrix[1][3], final_matrix[2][3]),
                        (init_pose.orientation.x, init_pose.orientation.y, init_pose.orientation.z, init_pose.orientation.w),
                        rospy.Time.now(),
                        "odom",
                        "map")
    else:
        br.sendTransform((0, 0, 0),
                        (0, 0, 0, 1),
                        rospy.Time.now(),
                        "odom",
                        "map")
    r.sleep()
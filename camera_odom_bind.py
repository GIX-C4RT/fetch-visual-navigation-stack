import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance
from nav_msgs.msg import Odometry
import roslib
import tf
from tf.transformations import *
import tf2_ros
import tf2_geometry_msgs
import numpy as np

init_pose = None
def reinit_pose(msg):
    global init_pose
    global Tpo
    init_pose = msg.pose.pose
    Tpo = tf_buffer.lookup_transform("camera_pose_frame",
                                       "camera_odom_frame", #source frame
                                       rospy.Time(0),
                                       rospy.Duration(5.0))

rospy.init_node('camera_odom_bind', anonymous=True)
r = rospy.Rate(25)
rospy.Subscriber("initialpose", PoseWithCovarianceStamped, reinit_pose)
br = tf.TransformBroadcaster()

tf_buffer = tf2_ros.Buffer(rospy.Duration(300.0)) #tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)

transform = tf_buffer.lookup_transform("odom",
                                       "base_link", #source frame
                                       rospy.Time(0),
                                       rospy.Duration(5.0)) #get the tf at first available time

listener = tf.TransformListener()
tm = translation_matrix([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
qm = quaternion_matrix([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
global_trans_matrix = concatenate_matrices(tm, qm)

Tpo = None
while not rospy.is_shutdown():
    if init_pose:
        tm = translation_matrix([init_pose.position.x, init_pose.position.y, init_pose.position.z])
        qm = quaternion_matrix([init_pose.orientation.x, init_pose.orientation.y, init_pose.orientation.z, init_pose.orientation.w])
        transform1 = concatenate_matrices(tm, qm)

        tm = translation_matrix([Tpo.transform.translation.x, Tpo.transform.translation.y, Tpo.transform.translation.z])
        qm = quaternion_matrix([Tpo.transform.rotation.x, Tpo.transform.rotation.y, Tpo.transform.rotation.z, Tpo.transform.rotation.w])
        transform2 = concatenate_matrices(tm, qm)
        global_trans_matrix = np.matmul(transform1, transform2)

    global_trans_quaternion = quaternion_from_matrix(global_trans_matrix)
    global_trans_translation = translation_from_matrix(global_trans_matrix)
    br.sendTransform(global_trans_translation,
                        global_trans_quaternion, 
                        rospy.Time.now(), 
                        "camera_odom_frame", 
                        "map")
    r.sleep()
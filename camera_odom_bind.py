import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance
from nav_msgs.msg import Odometry
import roslib
import tf
from tf.transformations import *
import tf2_ros
import tf2_geometry_msgs

init_pose = None
def reinit_pose(msg):
    global init_pose
    init_pose = msg.pose.pose

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

print transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z
while not rospy.is_shutdown():
    # br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
    #                 (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
    #                 rospy.Time.now(),
    #                 "camera_odom_frame",
    #                 "odom")
    if init_pose:
        try:
            (trans2,rot2) = listener.lookupTransform('/camera_odom_frame', '/camera_pose_frame', rospy.Time(0))
            global_trans = [init_pose.position.x - trans2[0], init_pose.position.y - trans2[1], 0]
            print "init: ", init_pose.position.x, init_pose.position.y
            print "op: ", trans2[0], trans2[1]
            print "global: ", global_trans
            br.sendTransform(tuple(global_trans),
                        (init_pose.orientation.x,init_pose.orientation.y,init_pose.orientation.z,init_pose.orientation.w), 
                        rospy.Time.now(), 
                        "camera_odom_frame", 
                        "map")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    else:
        br.sendTransform((transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z),
                            (transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w), 
                            rospy.Time.now(), 
                            "camera_odom_frame", 
                            "map")
        # br.sendTransform((transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z),
        #                     (0, 0, 0, 1), 
        #                     rospy.Time.now(), 
        #                     "camera_odom_frame", 
        #                     "map")
    r.sleep()
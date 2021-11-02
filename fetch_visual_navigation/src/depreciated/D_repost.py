import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance
from nav_msgs.msg import Odometry
import roslib
import tf

init_pose = None
br = tf.TransformBroadcaster()
def repost(msg):
    global br
    amcl = PoseWithCovarianceStamped()
    current_pose = PoseWithCovariance()
    if init_pose:
        # current_pose.pose.position.x = msg.pose.pose.position.x + init_pose.pose.position.x
        # current_pose.pose.position.y = msg.pose.pose.position.y + init_pose.pose.position.y
        # current_pose.pose.orientation = msg.pose.pose.orientation
        current_pose = msg.pose

        br.sendTransform((init_pose.pose.position.x, init_pose.pose.position.y, 0),
                    (init_pose.pose.orientation.x, init_pose.pose.orientation.y, init_pose.pose.orientation.z, init_pose.pose.orientation.w),
                    rospy.Time.now(),
                    "odom",
                    "map")
    else:
        current_pose = msg.pose

        print "publishing TF"
        br.sendTransform((0, 0, 0),
                    (0, 0, 0, 1),
                    rospy.Time.now(),
                    "odom",
                    "map")
    amcl.pose = current_pose
    # publisher.publish(amcl)

    
    # br.sendTransform((current_pose.pose.position.x, current_pose.pose.position.y, 0),
    #                 (current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w),
    #                 rospy.Time.now(),
    #                 "odom",
    #                 "map")

    # br.sendTransform((0, 0, 0),
    #                 (0, 0, 0, 1),
    #                 rospy.Time.now(),
    #                 "odom",
    #                 "map")

    # br.sendTransform((init_pose.pose.position.x, init_pose.pose.position.y, 0),
    #                 (init_pose.pose.orientation.x, init_pose.pose.orientation.y, init_pose.pose.orientation.z, init_pose.pose.orientation.w),
    #                 rospy.Time.now(),
    #                 "odom",
    #                 "map")

def reinit_pose(msg):
    global init_pose
    init_pose = msg.pose

rospy.init_node('dewey_nav', anonymous=True)
publisher = rospy.Publisher('amcl_pose', PoseWithCovarianceStamped, queue_size = 1)
rospy.Subscriber("/camera/odom/sample", Odometry, repost)
rospy.Subscriber("initialpose", PoseWithCovarianceStamped, reinit_pose)
r = rospy.Rate(25)
while not rospy.is_shutdown():
    r.sleep()
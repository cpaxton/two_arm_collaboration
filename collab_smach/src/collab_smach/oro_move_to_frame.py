import rospy
import roslib; roslib.load_manifest("collab_smach")
import smach
import smach_ros
import actionlib
import tf
import copy

# import predicator to let us see what's going on
import predicator_core.srv as pcs

# import predicator messages
from predicator_msgs.msg import *

# import joint/position messages
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

# import moveit messages
from moveit_msgs.msg import *
from moveit_msgs.srv import *

'''
MoveToFrameIK
Enables IK controllers and disables them
Broadcasts a command frame
Is terrible and should be replaced by something more robust
'''
class MoveToFrameIK(smach.state):
    def __init__(self, robot, frame):
        smach.State.__init__(self, outcomes=['success','failure'])
        self.robot = robot
        self.frame = frame
        self.deployer_name = "/gazebo/" + robot + "__deployer__"
        self.cmd_frame = "/" + robot + "/cmd"

        rospy.loginfo("Initializing IK node")

    def execute(self, userdata):

        tfl = tf.TransformListener()
        tfb = tf.TransformBroadcaster()

        runscript = rospy.ServiceProxy(self.deployer_name + "/run_script"

        print "Target frame: " + frame + ", getting transform..."
        tf_done = False

        while not tf_done:
            try:
                (trans, rot) = tfl.lookupTransform("/world", frame, rospy.Time(0))
                tf_done = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        print "Sending frame to " + self.cmd_frame

        for i = range(0,50):
            tfb.sendTransform(trans, rot, rospy.Time.now(0), self.cmd_frame, "/world")
            rospy.sleep(0.1)

        return 'success'

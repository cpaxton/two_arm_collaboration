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

# rtt services
from rtt_ros_msgs.srv import *
'''
MoveToFrameIK
Enables IK controllers and disables them
Broadcasts a command frame
Is terrible and should be replaced by something more robust
'''
class MoveToFrameNodeIK(smach.State):
    def __init__(self, robot, frame, ik_script, stop_script):
        smach.State.__init__(self, outcomes=['success','failure'])
        self.robot = robot
        self.frame = frame
        self.deployer_name = "/gazebo/" + robot + "__deployer__"
        self.cmd_frame = "/" + robot + "/cmd"
        self.start = ik_script
        self.stop = stop_script

        rospy.loginfo("Initializing IK node")

    def execute(self, userdata):

        tfl = tf.TransformListener()
        tfb = tf.TransformBroadcaster()

        runscript = rospy.ServiceProxy(self.deployer_name + "/run_script", RunScript)

        print "Target frame: " + self.frame + ", getting transform..."
        tf_done = False

        while not tf_done:
            try:
                (trans, rot) = tfl.lookupTransform("/world", self.frame, rospy.Time(0))
                tf_done = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        print "Sending frame to " + self.cmd_frame

        rospy.sleep(0.25)

        try:
            tfl.waitForTransform("world", self.cmd_frame, rospy.Time.now(), rospy.Duration(5.0))
        except tf.Exception, e:
            continue

        # try to start the controller
        for i in range (0, 10):
            result = runscript(self.start)
            if result.success == True:
                break
            rospy.sleep(0.05)

        if result.success == False:
            return 'failure'

        for i in range(0,500):
            tfb.sendTransform(trans, rot, rospy.Time.now(), self.cmd_frame, "/world")
            rospy.sleep(0.1)

        runscript(self.stop)

        return 'success'

import rospy
import roslib; roslib.load_manifest("collab_smach")
import smach
import smach_ros
import actionlib
import tf
import copy
import tf_conversions as tfc

# import predicator to let us see what's going on
from predicator_msgs.msg import *
import predicator_msgs.srv as pcs

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
WARNING: a lot of stuff in this program is hard coded and really should not be
'''
class MoveToFrameNodeIK(smach.State):
    def __init__(self, robot, frame, ik_script, stop_script, predicate=None, with_offset=None):
        smach.State.__init__(self, outcomes=['success','failure'])
        self.robot = robot
        self.frame = frame
        self.deployer_name = "/gazebo/" + robot + "__deployer__"
        self.cmd_frame = robot + "/cmd"
        self.wrist_frame = robot + "/wrist_palm_link"
        self.start = ik_script
        self.stop = stop_script
        self.predicate = predicate
        self.offset_frames = with_offset
        self.transform = None

        # use predicator to load settings
        self.ga = rospy.ServiceProxy("/predicator/get_assignment", pcs.GetAssignment)

        rospy.loginfo("Initializing IK node")

    '''
    get_frame_offset()
    '''
    def get_frame_offset(self, oframe1, oframe2):
        tf_done = False
        tfl = tf.TransformListener()

        while not tf_done:
            try:
                (trans1, rot1) = tfl.lookupTransform(oframe1, oframe2, rospy.Time(0))
                tf_done = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        
        tf_done = False

        while not tf_done:
            try:
                (trans2, rot2) = tfl.lookupTransform("/world", self.frame, rospy.Time(0))
                tf_done = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        print (trans1, rot1)
        print (trans2, rot2)

        f1 = tfc.fromTf((trans1, rot1))
        f2 = tfc.fromTf((trans2, rot2))

        frame = f2 * f1

        self.transform = tfc.toTf(frame)

        tfb = tf.TransformBroadcaster()
        (t, r) = self.transform
        tfb.sendTransform(t, r, rospy.Time.now(), "EX", "/world")
        self.frame = "EX"

    def execute(self, userdata):

        tfl = tf.TransformListener()
        tfb = tf.TransformBroadcaster()

        if not self.predicate == None:
            print self.predicate

            resp = self.ga(self.predicate)

            print resp

            # choose a random frame
            idx = random.randrange(len(resp.values))
            self.frame = resp.values[idx].params[0]

            print "Updating frame via predicate: " + self.frame

        if not self.offset_frames == None:
            (oframe1, oframe2) = self.offset_frames
            self.get_frame_offset(oframe1, oframe2)

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

        # try to start the controller
        for i in range (0, 10):
            tfb.sendTransform(trans, rot, rospy.Time.now(), self.cmd_frame, "/world")
            rospy.sleep(0.1)
            print "Attempt %d"%(i)
            result = runscript(self.start)
            rospy.sleep(0.05)

        if result.success == False:
            return 'failure'

        waiting = True

        while waiting:

            tfb.sendTransform(trans, rot, rospy.Time.now(), self.cmd_frame, "/world")

            tf_done = False
            while not tf_done:
                try:
                    (rtrans, rrot) = tfl.lookupTransform("/world", self.wrist_frame, rospy.Time(0))
                    tf_done = True
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

                frame_coors = trans + rot
                robot_coors = rtrans + rrot
                diff = [abs(i[0] - i[1]) for i in zip(frame_coors, robot_coors)]

                waiting = False
                for i in diff:
                    if i > 0.1:
                        waiting = True

            tfb.sendTransform(trans, rot, rospy.Time.now(), self.cmd_frame, "/world")
            rospy.sleep(0.1)

        rospy.sleep(0.25)
        print "Stopping IK script"
        runscript(self.stop)
        rospy.sleep(0.25)

        return 'success'

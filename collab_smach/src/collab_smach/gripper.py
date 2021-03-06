import rospy
import roslib; roslib.load_manifest("collab_smach")
import smach
import smach_ros

# import predicator to let us see what's going on
import predicator_msgs.srv as pcs
from predicator_msgs.msg import *

from oro_barrett_msgs.msg import BHandCmd

hand_opened = BHandCmd()
hand_closed = BHandCmd()

# set up the hand open position
hand_opened.mode = [3, 3, 3, 3]
hand_opened.cmd = [0.5, 0.5, 0.5, 0]

# set up the hand closed position
hand_closed.mode = [3, 3, 3, 3]
hand_closed.cmd = [2.0, 2.0, 2.0, 0]

'''
CloseGripperNode
'''
class CloseGripperNode(smach.State):
    def __init__(self,robot,attach=None):
        smach.State.__init__(self, outcomes=['success','failure'])
        self.robot = robot
        self.obj = attach

        # use predicator to load settings
        ga = rospy.ServiceProxy("/predicator/get_assignment", pcs.GetAssignment)
        statement = PredicateStatement()
        statement.predicate = "gripper_topic"
        statement.params[1] = robot
        statement.params[0] = "*"
        resp = ga(statement)

        if resp.found:
            self.topic_set = True
            self.pub = rospy.Publisher(resp.values[0].params[0], BHandCmd)
        else:
            self.topic_set = False

    def execute(self, userdata):
        if self.topic_set == True:
            for i in range(0,10):
                self.pub.publish(hand_closed)
                rospy.sleep(0.1)
            return 'success'
        else:
            return 'failure'

'''
OpenGripperNode
'''
class OpenGripperNode(smach.State):
    def __init__(self,robot,detach=None):
        smach.State.__init__(self, outcomes=['success','failure'])
        self.robot = robot
        self.obj = detach

        # use predicator to load settings
        ga = rospy.ServiceProxy("/predicator/get_assignment", pcs.GetAssignment)
        statement = PredicateStatement()
        statement.predicate = "gripper_topic"
        statement.params[1] = robot
        statement.params[0] = "*"
        resp = ga(statement)

        if resp.found:
            self.topic_set = True
            self.pub = rospy.Publisher(resp.values[0].params[0], BHandCmd)
        else:
            self.topic_set = False

    def execute(self, userdata):
        if self.topic_set == True:
            for i in range(0,10):
                self.pub.publish(hand_opened)
                rospy.sleep(0.1)
            return 'success'
        else:
            return 'failure'

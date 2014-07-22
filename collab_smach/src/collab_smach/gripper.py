import rospy
import roslib; roslib.load_manifest("collab_smach")
import smach
import smach_ros

# set up the hand open position
self.hand_opened.mode = [3, 3, 3, 3]
self.hand_opened.cmd = [0.5, 0.5, 0.5, 0]

# set up the hand closed position
self.hand_closed.mode = [3, 3, 3, 3]
self.hand_closed.cmd = [2.0, 2.0, 2.0, 0]

# import predicator to let us see what's going on
import predicator_core.srv as pcs

'''
CloseGripperNode
'''
class CloseGripperNode(smach.State):
    def __init__(self,robot):
        self.robot = robot

        #self.pub = rospy.Publisher()

    def execute(self):
        return 'success'

'''
OpenGripperNode
'''
class OpenGripperNode(smach.State):
    def __init__(self,robot):
        self.robot = robot

        #self.pub = rospy.Publisher()

    def execute(self):
        return 'success'

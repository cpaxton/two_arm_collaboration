import rospy
import roslib; roslib.load_manifest("collab_smach")
import smach
import smach_ros

# import predicator to let us see what's going on
import predicator_core.srv as pcs

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
    def __init__(self,robot):
        smach.State.__init__(self, outcomes=['success','failure'])
        self.robot = robot

        #self.pub = rospy.Publisher()

    def execute(self):
        return 'success'

'''
OpenGripperNode
'''
class OpenGripperNode(smach.State):
    def __init__(self,robot):
        smach.State.__init__(self, outcomes=['success','failure'])
        self.robot = robot

        #self.pub = rospy.Publisher()

    def execute(self):
        return 'success'

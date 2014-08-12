import rospy
import roslib; roslib.load_manifest("collab_smach")
import smach
import smach_ros

import predicator_msgs.srv as ps
from predicator_msgs.msg import *

'''
TestPredicateNode
Returns true if a predicate was found, false if not found, and unknown if there was an error.
'''
class TestPredicateNode(smach.State):
    def __init__(self,predicate):
        smach.State.__init__(self, outcomes=['true', 'false', 'unknown'])


    def execute(self, userdata):

        pass

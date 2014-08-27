import rospy
import roslib; roslib.load_manifest("collab_smach")
import smach
import smach_ros

import predicator_msgs.srv as ps
from predicator_msgs.msg import *
from preciator_planning.srv import *

'''
PredicateMoveNode
Performs a predicate-based move, for a specified arm and group.
Uses the sampling-based planner implemented in predicator_planning.
'''
class PredicateMoveNode(smach.State):
    def __init__(self,predicate):
        smach.State.__init__(self, outcomes=['true', 'false', 'unknown'])

        self.predicate = predicate
        self.call = rospy.ServiceProxy("/predicator/plan", PredicatePlan)

    def execute(self, userdata):

        resp = self.call(self.predicate)
            
        if resp.found:
            return 'true'
        else:
            return 'false'

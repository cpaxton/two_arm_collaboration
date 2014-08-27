import rospy
import roslib; roslib.load_manifest("collab_smach")
import smach
import smach_ros

import predicator_msgs.srv as ps
from predicator_msgs.msg import *
from predicator_planning.srv import *

'''
PredicateMoveNode
Performs a predicate-based move, for a specified arm and group.
Uses the sampling-based planner implemented in predicator_planning.
'''
class PredicateMoveNode(smach.State):
    def __init__(self,robot,req_true,req_false,goal_true,goal_false,group="arm"):
        smach.State.__init__(self, outcomes=['success', 'failure', 'incomplete'])

        self.req = PredicatePlanRequest()
        req.robot = robot
        req.required_true = req_true
        req.required_false = req_false
        req.goal_true = goal_true
        req.goal_false = goal_false
        req.group = group

        self.predicate = predicate
        self.call = rospy.ServiceProxy("/predicator/plan", PredicatePlan)

    def execute(self, userdata):

        resp = self.call(self.req)

        if resp.found == True:
            return 'success'
        else:
            return 'incomplete'
            

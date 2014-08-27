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
        self.robot = robot
        self.required_true = req_true
        self.required_false = req_false
        self.goal_true = goal_true
        self.goal_false = goal_false
        self.group = group

        self.call = rospy.ServiceProxy("predicator/plan", PredicatePlan)

    def execute(self, userdata):

        try:
            resp = self.call(self.req)
            if resp.found == True:
                return 'success'
            else:
                return 'incomplete'

            # send to appropriate topic

        except Exception as e:
            print e
            return 'failure'


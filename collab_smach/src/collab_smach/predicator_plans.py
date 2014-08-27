import rospy
import roslib; roslib.load_manifest("collab_smach")
import smach
import smach_ros

import predicator_msgs.srv as ps
from predicator_msgs.msg import *
from predicator_planning.srv import *
from control_msgs.msg import *
import actionlib

'''
PredicateMoveNode
Performs a predicate-based move, for a specified arm and group.
Uses the sampling-based planner implemented in predicator_planning.
'''
class PredicateMoveNode(smach.State):
    def __init__(self,robot,req_true,req_false,goal_true,goal_false,group="arm"):
        smach.State.__init__(self, outcomes=['success', 'failure', 'incomplete'])

        self.req = PredicatePlanRequest()
        self.req.robot = robot
        self.req.required_true = req_true
        self.req.required_false = req_false
        self.req.goal_true = goal_true
        self.req.goal_false = goal_false
        self.req.group = group

        self.call = rospy.ServiceProxy("predicator/plan", PredicatePlan)



    def execute(self, userdata):

        try:
            resp = self.call(self.req)

            # send to appropriate topic
            client = actionlib.SimpleActionClient('/gazebo/traj_rml/action', control_msgs.msg.FollowJointTrajectoryAction)

            goal = FollowJointTrajectoryGoal()
            goal.trajectory = resp.path
            
            client.send_goal(goal)
            res = client.wait_for_result()

            print res

            if resp.found == True:
                return 'success'
            else:
                return 'incomplete'


        except Exception as e:
            print e
            return 'failure'


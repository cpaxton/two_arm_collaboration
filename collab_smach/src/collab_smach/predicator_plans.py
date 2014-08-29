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
    def __init__(self,robot,req_true,req_false,goal_true,goal_false,action,group="arm"):
        smach.State.__init__(self, outcomes=['success', 'failure', 'incomplete'])

        self.req = PredicatePlanRequest()
        self.req.robot = robot
        self.req.required_true = req_true
        self.req.required_false = req_false
        self.req.goal_true = goal_true
        self.req.goal_false = goal_false
        self.req.group = group
        self.action = action

        self.call = rospy.ServiceProxy("predicator/plan", PredicatePlan)

    def execute(self, userdata):

        try:
            resp = self.call(self.req)

            rospy.loginfo("Adding timestamps to trajectory; hack to get around issues with traj_rml.")
            next_t = 0
            for point in resp.path.points:
                point.time_from_start.secs = next_t
                next_t = next_t + 2

            print resp

            # send to appropriate topic
            client = actionlib.SimpleActionClient(self.action, control_msgs.msg.FollowJointTrajectoryAction)

            goal = FollowJointTrajectoryGoal()

            goal.trajectory = resp.path
            
            rospy.loginfo("Sending trajectory to trajectory action...")

            client.wait_for_server()
            client.send_goal(goal)
            res = client.wait_for_result()

            print res

            checker = rospy.ServiceProxy("predicator/get_assignment", ps.GetAssignment);

            # make sure that we give predicator time to catch up
            rospy.sleep(0.50)

            # use predicator to check goals
            for goal in self.req.goal_true:
                check_result = checker(goal)
                if check_result.found == False:
                    rospy.loginfo("Predicator true goal still unmet!")
                    print goal
                    return 'incomplete'

            for goal in self.req.goal_false:
                check_result = checker(goal)
                if check_result.found == True:
                    rospy.loginfo("Predicator galse goal still unmet!")
                    print goal
                    return 'incomplete'

            if resp.found == True: #and res.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
                return 'success'
            else:
                return 'incomplete'


        except Exception as e:
            print e
            return 'failure'


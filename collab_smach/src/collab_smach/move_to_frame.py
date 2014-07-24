import rospy
import roslib; roslib.load_manifest("collab_smach")
import smach
import smach_ros
import actionlib
import tf
import copy

# import predicator to let us see what's going on
import predicator_core.srv as pcs

# import predicator messages
from predicator_msgs.msg import *

# import joint/position messages
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

# import moveit messages
from moveit_msgs.msg import *
from moveit_msgs.srv import *


'''
MoveToFrameNode

This is a simple SMACH node that tells a certain robot to move to a frame.
Robot movement parameters need to be set up correctly -- it will use MoveGroup to do motion planning.
The difference between this node and the above is that this node will alter the allowed collisions matrix,
disabiling collisions with whatever object we might run into because we want to grab it.
'''
class MoveToFrameNode(smach.State):
    def __init__(self,robot,frame,objs=None):
        smach.State.__init__(self, outcomes=['success','failure','moveit_error'])
        self.robot = robot
        self.frame = frame
        self.objs = objs

        rospy.loginfo("Initializing move to frame node")

        # use predicator to load settings
        self.ga = rospy.ServiceProxy("/predicator/get_assignment", pcs.GetAssignment)
        statement = PredicateStatement()
        statement.predicate = "move_group_namespace"
        statement.params[1] = robot
        statement.params[0] = "*"
        resp = self.ga(statement)

        if resp.found:
            self.ns = resp.values[0].params[0]
        else:
            # default namespace
            self.ns = "/move_group"

        statement = PredicateStatement()
        statement.predicate = "robot_namespace"
        statement.params[1] = robot
        statement.params[0] = "*"
        resp = self.ga(statement)

        self.robot_ns = resp.values[0].params[0]
        self.js_sub = rospy.Subscriber(self.robot_ns + "/wam/joint_states", sensor_msgs.msg.JointState, self.joint_state_cb)

        statement = PredicateStatement()
        statement.predicate = "planning_scene_topic"
        statement.params[1] = robot
        statement.params[0] = "*"
        resp = self.ga(statement)

        self.ps_pub = rospy.Publisher(resp.values[0].params[0], moveit_msgs.msg.PlanningScene)

        rospy.loginfo("Planning scene topic: %s", resp.values[0].params[0])
        rospy.loginfo("Move group location: %s", self.ns)
        rospy.loginfo("Joints topic: %s", self.robot_ns + "/wam/joint_states")

    '''
    joint_state_cb()
    keep up to date data on joints
    '''
    def joint_state_cb(self, msg):
        self.js = msg

    def execute(self, userdata):

        self.client = actionlib.SimpleActionClient(self.ns, MoveGroupAction)

        ps_proxy = rospy.ServiceProxy(self.robot_ns + "/get_planning_scene", moveit_msgs.srv.GetPlanningScene)

        planning_options = PlanningOptions()
        planning_options.plan_only = False
        planning_options.replan = False
        planning_options.replan_attempts = 5
        planning_options.replan_delay = 2.0

        if not self.objs == None:
            ps_req = PlanningSceneComponents()
            ps_req.components = moveit_msgs.msg.PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            ps = ps_proxy(components=ps_req).scene
            ps_new_acm = self.update_collision_matrix(objs=self.objs, cm=copy.deepcopy(ps.allowed_collision_matrix))

            #print ps_new_acm
            self.ps_pub.publish(ps_new_acm)

        motion_req = MotionPlanRequest()

        motion_req.start_state.joint_state = self.js
        motion_req.workspace_parameters.header.frame_id = "world"
        motion_req.workspace_parameters.max_corner.x = 2.0
        motion_req.workspace_parameters.max_corner.y = 2.0
        motion_req.workspace_parameters.max_corner.z = 2.0
        motion_req.workspace_parameters.min_corner.x = -2.0
        motion_req.workspace_parameters.min_corner.y = -2.0
        motion_req.workspace_parameters.min_corner.z = -2.0

        # create the goal constraints
        motion_req.goal_constraints.append(self.getConstraints(self.frame))
        motion_req.group_name = "arm"
        motion_req.num_planning_attempts = 10
        motion_req.allowed_planning_time = 5.0
        motion_req.planner_id = "RTTstarkConfigDefault"
        
        self.goal = MoveGroupGoal()
        self.goal.planning_options = planning_options
        self.goal.request = motion_req

        #print self.goal
        print "Sending request..."

        self.client.send_goal(self.goal)
        self.client.wait_for_result()
        res = self.client.get_result()

        print "Done: " + str(res.error_code.val)

        if not self.objs == None:
            print "Reverting planning scene/allowed collision matrix"
            ps.is_diff = True
            self.ps_pub.publish(ps)

        if res.error_code.val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
            return 'success'
        elif res.error_code.val >= -4:
            return 'moveit_error'
        else:
            return 'failure'

    '''
    getConstraints()
    Helper function to generate goal constraints from a position
    '''
    def getConstraints(self, frame):
        tfl = tf.TransformListener()
        
        # compute ik for this position
        srv = rospy.ServiceProxy(self.robot_ns + "/compute_ik", moveit_msgs.srv.GetPositionIK)

        print "Target frame: " + frame + ", getting transform..."
        tf_done = False

        while not tf_done:
            try:
                (trans, rot) = tfl.lookupTransform("/world", frame, rospy.Time(0))
                tf_done = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        p = geometry_msgs.msg.PoseStamped()
        p.pose.position.x = trans[0]
        p.pose.position.y = trans[1]
        p.pose.position.z = trans[2]
        p.pose.orientation.x = rot[0]
        p.pose.orientation.y = rot[1]
        p.pose.orientation.z = rot[2]
        p.pose.orientation.w = rot[3]
        p.header.frame_id = "/world"

        ik_req = moveit_msgs.msg.PositionIKRequest()
        ik_req.robot_state.joint_state = self.js
        ik_req.avoid_collisions = True
        ik_req.timeout = rospy.Duration(3.0)
        ik_req.attempts = 5
        ik_req.group_name = "arm"
        ik_req.pose_stamped = p

        print "Getting IK position..."
        ik_resp = srv(ik_req)

        print "IK RESULT ERROR CODE = %d"%(ik_resp.error_code.val)

        ###############################
        # now create the goal based on inverse kinematics

        goal = Constraints()

        for i in range(0,len(ik_resp.solution.joint_state.name)):
            print ik_resp.solution.joint_state.name[i]
            print ik_resp.solution.joint_state.position[i]
            joint = JointConstraint()
            joint.joint_name = ik_resp.solution.joint_state.name[i]
            joint.position = ik_resp.solution.joint_state.position[i] 
            joint.tolerance_below = 0.001
            joint.tolerance_above = 0.001
            joint.weight = 1.0
            goal.joint_constraints.append(joint)

        return goal


    '''
    update_collision_matrix()
    look at the allowed collision matrix between object and robot
    '''
    def update_collision_matrix(self, objs, cm):

        ps = moveit_msgs.msg.PlanningScene()
        ps.is_diff = True
        ps.allowed_collision_matrix = cm

        #if not obj == None:
        for obj in objs:

            #comps_robot_req = PredicateStatement()
            #comps_robot_req.predicate = "hand_component"
            #comps_robot_req.params[1] = self.robot
            #comps_robot_req.params[0] = "*"

            comps_obj_req = PredicateStatement()
            comps_obj_req.predicate = "component"
            comps_obj_req.params[1] = obj
            comps_obj_req.params[0] = "*"

            #robot_comps = self.ga(comps_robot_req)
            obj_comps = self.ga(comps_obj_req)

            #for i in range(0, len(robot_comps.values)):
            #    for j in range (0, len(obj_comps.values)):
            #        print "adding (%s, %s) to allowed collisions"%(robot_comps.values[i].params[0],obj_comps.values[j].params[0])

            old_len = len(ps.allowed_collision_matrix.entry_names)
            #for i in range(0, len(robot_comps.values)):
            #    ps.allowed_collision_matrix.entry_names.append(robot_comps.values[i].params[0])
            for i in range(0, len(obj_comps.values)):
                ps.allowed_collision_matrix.entry_names.append(obj_comps.values[i].params[0])

            new_len = len(ps.allowed_collision_matrix.entry_names)

            print ps.allowed_collision_matrix.entry_names
            print "old length = " + str(old_len)
            print "new length = " + str(new_len)

            # add to old rows
            for i in range(0, old_len):
                for j in range(old_len, new_len):
                    ps.allowed_collision_matrix.entry_values[i].enabled.append(False)
                    print "row %d = %d"%(i,len(ps.allowed_collision_matrix.entry_values[i].enabled))

            # add new rows
            for i in range(old_len, new_len):
                entry = moveit_msgs.msg.AllowedCollisionEntry()
                for j in range(0, new_len):
                    entry.enabled.append(False)
                print "row %d = %d"%(i,len(entry.enabled))
                ps.allowed_collision_matrix.entry_values.append(entry)
                print "%d rows"%(len(ps.allowed_collision_matrix.entry_values))

        return ps

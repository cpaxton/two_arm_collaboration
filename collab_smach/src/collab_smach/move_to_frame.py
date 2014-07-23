import rospy
import roslib; roslib.load_manifest("collab_smach")
import smach
import smach_ros
import actionlib
import tf

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
getConstraints()
Helper function to generate goal constraints from a position
'''
def getConstraints(ee_link, frame):
    tfl = tf.TransformListener()

    goal = Constraints()
    goal.name = "goal"

    position = PositionConstraint()
    orientation = OrientationConstraint()
    joint0 = JointConstraint()

    position.constraint_region = BoundingVolume()
    position.weight = 1.0
    position.link_name = ee_link

    print ee_link
    print frame
    #tfl.waitForTransform(ee_link, frame, rospy.Time(0), rospy.Time(2.0))
    tf_done = False

    while not tf_done:
        try:
            (trans, rot) = tfl.lookupTransform(ee_link, frame, rospy.Time(0))
            tf_done = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    print (trans, rot)

    p = geometry_msgs.msg.Pose()
    p.position.x = trans[0]
    p.position.y = trans[1]
    p.position.z = trans[2]

    region = shape_msgs.msg.SolidPrimitive()
    region.type = shape_msgs.msg.SolidPrimitive.SPHERE
    region.dimensions.append(0.1)

    position.constraint_region.primitive_poses.append(p)

    #goal.position_constraints.append(position)
    #goal.orientation_constraints.append(orientation)

    return goal
        

'''
MoveToFrameNode

This is a simple SMACH node that tells a certain robot to move to a frame.
Robot movement parameters need to be set up correctly -- it will use MoveGroup to do motion planning.
'''
class MoveToFrameNode(smach.State):
    def __init__(self,robot,frame):
        smach.State.__init__(self, outcomes=['success','failure','moveit_error'])
        self.robot = robot
        self.frame = frame

        rospy.loginfo("Initializing move to frame node")

        # use predicator to load settings
        ga = rospy.ServiceProxy("/predicator/get_assignment", pcs.GetAssignment)
        statement = PredicateStatement()
        statement.predicate = "move_group_namespace"
        statement.params[1] = robot
        statement.params[0] = "*"
        resp = ga(statement)

        if resp.found:
            self.ns = resp.values[0].params[0]
        else:
            # default namespace
            self.ns = "/move_group"


        rospy.loginfo("Move group location: %s", self.ns)

        self.client = actionlib.SimpleActionClient(self.ns, MoveGroupAction)

    def execute(self, userdata):

        return 'success'

'''
MoveToObjectFrameNode

This is a simple SMACH node that tells a certain robot to move to a frame.
Robot movement parameters need to be set up correctly -- it will use MoveGroup to do motion planning.
The difference between this node and the above is that this node will alter the allowed collisions matrix,
disabiling collisions with whatever object we might run into because we want to grab it.
'''
class MoveToObjectFrameNode(smach.State):
    def __init__(self,robot,obj):
        smach.State.__init__(self, outcomes=['success','failure','moveit_error'])
        self.robot = robot
        self.obj = obj

        rospy.loginfo("Initializing move to frame node")

        # use predicator to load settings
        ga = rospy.ServiceProxy("/predicator/get_assignment", pcs.GetAssignment)
        statement = PredicateStatement()
        statement.predicate = "move_group_namespace"
        statement.params[1] = robot
        statement.params[0] = "*"
        resp = ga(statement)

        if resp.found:
            self.ns = resp.values[0].params[0]
        else:
            # default namespace
            self.ns = "/move_group"

        rospy.loginfo("Move group location: %s", self.ns)

    def execute(self, userdata):

        self.client = actionlib.SimpleActionClient(self.ns, MoveGroupAction)

        planning_options = PlanningOptions()
        planning_options.plan_only = False
        planning_options.replan = True
        planning_options.replan_attempts = 5
        planning_options.replan_delay = 1.0

        motion_req = MotionPlanRequest()
        
        motion_req.allowed_planning_time = 5.0
        motion_req.workspace_parameters.header.frame_id = "world"
        motion_req.workspace_parameters.max_corner.x = 2.0
        motion_req.workspace_parameters.max_corner.y = 2.0
        motion_req.workspace_parameters.max_corner.z = 2.0
        motion_req.workspace_parameters.min_corner.x = -2.0
        motion_req.workspace_parameters.min_corner.y = -2.0
        motion_req.workspace_parameters.min_corner.z = -2.0

        # create the goal constraints
        motion_req.goal_constraints.append(getConstraints("wam2/wrist_palm_link","location1"))
        
        self.goal = MoveGroupGoal()
        self.goal.planning_options = planning_options
        self.goal.request = motion_req

        self.client.send_goal(self.goal)
        self.client.wait_for_result()
        
        print self.client.get_result()

        return 'success'

import rospy
import roslib; roslib.load_manifest("collab_smach")
import smach
import smach_ros
import actionlib

# import predicator to let us see what's going on
import predicator_core.srv as pcs

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

        # use predicator to load settings
        ga = rospy.ServiceProxy("/predicator/get_assignment", pcs.GetAssignment)
        statement = PredicateStatement()
        statement.predicate = "move_group_namespace"
        statement.params[1] = robot
        statement.params[0] = "*"
        resp = ga(statement)

        if resp.found:
            self.ns = resp.values[0].params[1]
        else:
            # default namespace
            self.ns = "/move_group"

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

    def execute(self, userdata):
        return 'success'

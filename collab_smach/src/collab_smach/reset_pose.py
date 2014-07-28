import rospy
import roslib; roslib.load_manifest("collab_smach")
import smach
import smach_ros

from trajectory_msgs.msg import JointTrajectoryPoint

# import predicator to let us see what's going on
import predicator_msgs.srv as pcs

home_pt = JointTrajectoryPoint()
home_pt.positions = [0, -1.5707, 0, 3.1415, 0, -1.5705, 0]

'''
ResetPoseNode
This is a forced reset -- no motion planning, just tell it to go there
'''
class ResetPoseNode(smach.State):
    def __init__(self,topic, pt=home_pt):
        smach.State.__init__(self, outcomes=['success','failure'])
        self.pt = pt
        self.topic = topic

        #statement = PredicateStatement()
        #statement.predicate = "robot_namespace"
        #statement.params[1] = robot
        #statement.params[0] = "*"
        #resp = self.ga(statement)
        #self.robot_ns = resp.values[0].params[0]

    def execute(self, userdata):

        pub = rospy.Publisher(self.topic, JointTrajectoryPoint)
        pub.publish(self.pt)

        return 'success'


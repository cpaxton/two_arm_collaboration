import rospy
import roslib; roslib.load_manifest("collab_smach")
import smach
import smach_ros

from trajectory_msgs import JointTrajectoryPoint

# import predicator to let us see what's going on
import predicator_core.srv as pcs

home_pt = JointTrajectoryPoint()
home_pt.positions = [0, -1.5707, 0, 3.1415, 0, -1.5705, 0]

class ResetPoseNode(smach.State):
    def __init__(self,topic,pt=home_pt):
        self.pt = pt
        self.pub = rospy.Publisher(topic, JointTrajectoryPoint)

    def execute(self):
        self.pub.publish(pt)
        return 'success'


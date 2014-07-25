import rospy
import roslib; roslib.load_manifest("collab_smach")
import smach
import smach_ros

'''
TimedSleepNode
Tells execution to sleep for a certain amount of time.
'''
class TimedSleepNode(smach.State):
    def __init__(self, time):
        smach.State.__init__(self, outcomes=['success'])
        self.time = time

    def execute(self, userdata):
        rospy.sleep(self.time)

        return 'success'

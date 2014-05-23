#!/usr/bin/env python

import roslib; roslib.load_manifest('peg_assist_demo')
import rospy
import smach
import smach_ros

# import simple action state
from smach_ros import SimpleActionState

# import message types for the different possible actions
from peg_assist_demo.msg import *

human_arm = 'wam'
auto_arm = 'wam2'

class WaitForRing (smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])
        self.result = 0

    def execute(self, userdata):
        rospy.loginfo('Waiting for Ring...')
        return 'success'


class GrabRing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.loginfo('Attempting to grab ring...')
        return 'success'

class WaitForRelease(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.loginfo('Waiting for ring to be released...')
        return 'success'

class RingToPeg(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.loginfo('Moving ring to peg...')
        return 'success'

class DropRing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.loginfo('Dropping ring...')
        return 'success'

if __name__ == '__main__':
    rospy.init_node('smach_peg_task')

    sm = smach.StateMachine(outcomes=['RING_TRANSFER_COMPLETE'])

    with sm:
            smach.StateMachine.add('WaitForRing', WaitForRing(),
                    transitions={'success':'GrabRing'})
            smach.StateMachine.add('GrabRing', GrabRing(),
                    transitions={'success':'WaitForRelease'})
            smach.StateMachine.add('WaitForRelease', WaitForRelease(),
                    transitions={'success':'RingToPeg'})
            smach.StateMachine.add('RingToPeg', RingToPeg(),
                    transitions={'success':'DropRing'})
            smach.StateMachine.add('DropRing', DropRing(),
                    transitions={'success':'RING_TRANSFER_COMPLETE'})

    outcome = sm.execute()

    print outcome


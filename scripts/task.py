#!/usr/bin/env python

import roslib; roslib.load_manifest('peg_assist_demo')
import rospy
import smach
import smach_ros

# import simple action state
from smach_ros import SimpleActionState

# import message types for the different possible actions
from peg_assist_demo.msg import *
from dmp_action.srv import *
from dmp.msg import *

human_arm = 'wam'
auto_arm = 'wam2'
goal_ring = 'ring1'

class WaitForRing (smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failure'])
        self.result = 0

    def execute(self, userdata):
        rospy.loginfo('Waiting for Ring...')
        client = actionlib.SimpleActionClient('WaitForRing',
                peg_assist_demo.msg)
        client.wait_for_server()
        
        # set up the goal for the action
        goal = peg_assist_demo.msg.WaitForRingGoal();
        goal.arm_id = auto_arm
        goal.ring_id = goal_ring
        goal.distance_threshold = 0.5 # should be learned

        # send the goal along
        client.send_goal(goal)
        client.wait_for_result()
        res =  client.get_result()
        if res.ring_available == 1 :
            return 'success'
        else :
            return 'failure'

class FindRing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['in_range','out_of_range'])

    def execute(self, userdata):
        rospy.loginfo('Finding ring and setting location data...')

        return 'in_range'

class ResetPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reset'])

    def execute(self, userdata):
        rospy.loginfo('Reset position of arm to home')

        return 'reset'

# for now: compute location of ring, play 
class GrabRing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failure'])

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
        rospy.loginfo('Waiting for gripper control service...')

        rospy.loginfo('Dropping ring...')
        return 'success'

if __name__ == '__main__':
    rospy.init_node('smach_peg_task')

    sm = smach.StateMachine(outcomes=['RING_TRANSFER_COMPLETE'])

    with sm:
            smach.StateMachine.add('WaitForRing', WaitForRing(),
                    transitions={
                        'success':'FindRing',
                        'failure':'WaitForRing'})
            smach.StateMachine.add('FindRing', FindRing(),
                    transitions={
                        'in_range':'GrabRing',
                        'out_of_range':'WaitForRing'})
            smach.StateMachine.add('GrabRing', GrabRing(),
                    transitions={
                        'success':'WaitForRelease',
                        'failure': 'ResetPosition1'})
            smach.StateMachine.add('ResetPosition1', ResetPosition(),
                    transitions={
                        'reset':'FindRing'})
            smach.StateMachine.add('WaitForRelease', WaitForRelease(),
                    transitions={'success':'RingToPeg'})
            smach.StateMachine.add('RingToPeg', RingToPeg(),
                    transitions={'success':'DropRing'})
            smach.StateMachine.add('DropRing', DropRing(),
                    transitions={'success':'RING_TRANSFER_COMPLETE'})

    # Create SMACH introspection server
    sis = smach_ros.IntrospectionServer('peg_task_introspection_server', sm, '/SM_ROOT')
    sis.start()

    move_ring_file = rospy.get_param("~move_ring_file")
    grab_ring_file = rospy.get_param("~grab_ring_file")
    
    print "TASK >>> services"

    # load the YAML file describing the move to peg ring motion
    rospy.wait_for_service('move_ring_motion/load_file')
    loader = rospy.ServiceProxy('move_ring_motion/load_file', LoadFile)
    
    # load the YAML file describing the grab ring motion
    rospy.wait_for_service('grab_ring_motion/load_file')
    loader = rospy.ServiceProxy('grab_ring_motion/load_file', LoadFile)

    print "TASK >>> Loaded files?"

    # execute the state machine
    outcome = sm.execute()

    print "TASK >>> Done state machine?"

    rospy.spin()
    sis.stop()

    print outcome


#!/usr/bin/env python

import roslib; roslib.load_manifest('peg_assist_demo')
import rospy
import smach
import smach_ros

import collab_smach


if __name__ == '__main__':
    rospy.init_node('smach_peg_task_iros2014')

    sm = smach.StateMachine(outcomes=['DONE'])

    with sm:
        smach.StateMachine.add('Open1', collab_smach.OpenGripperNode('wam'),
                transitions={
                    'success': 'Open2',
                    'failure': 'ERROR'})
        smach.StateMachine.add('Open2', collab_smach.OpenGripperNode('wam'),
                transitions={
                    'success': 'MoveToRing',
                    'failure': 'ERROR'})
        smach.StateMachine.add('MoveToRing', collab_smach.MoveToObjectFrameNode('wam','ring1'),
                transitions={
                    'success': 'GrabRing',
                    'moveit_error': 'MovetoRing'})


    # Create SMACH introspection server
    sis = smach_ros.IntrospectionServer('peg_task_introspection_server', sm, '/SM_ROOT')
    sis.start()

    # execute the state machine
    outcome = sm.execute()

    print "TASK >>> Done state machine?"

    rospy.spin()
    sis.stop()

    print outcome


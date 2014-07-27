#!/usr/bin/env python

import roslib; roslib.load_manifest('peg_assist_demo')
import rospy
import smach
import smach_ros

import collab_smach

from predicator_msgs.msg import *


if __name__ == '__main__':
    rospy.init_node('smach_peg_task_iros2014')

    sm = smach.StateMachine(outcomes=['DONE','ERROR'])

    scripts = rospy.get_param("~script_root")
    arm1_ik = scripts + "arm1_enable_ik.ops"
    arm2_ik = scripts + "arm2_enable_ik.ops"
    arm1_stop = scripts + "arm1_enable_joint.ops"
    arm2_stop = scripts + "arm2_enable_joint.ops"

    print "IK scripts location:" + scripts
    print "Example IK script: " + arm1_ik

    reachablePredicate = PredicateStatement()
    reachablePredicate.predicate = "inverse_reachable"
    reachablePredicate.num_params = 2
    reachablePredicate.params[0] = '*'
    reachablePredicate.params[1] = 'wam2'

    grabRingPredicate = PredicateStatement()
    grabRingPredicate.predicate = 'grasp_point'
    grabRingPredicate.num_params = 2
    grabRingPredicate.params[0] = '*'
    grabRingPredicate.params[1] = 'ring1'

    print reachablePredicate
    print grabRingPredicate

    with sm:
        smach.StateMachine.add('Open1', collab_smach.OpenGripperNode('wam'),
                transitions={
                    'success': 'Open2',
                    'failure': 'ERROR'})
        smach.StateMachine.add('Open2', collab_smach.OpenGripperNode('wam2'),
                transitions={
                    'success': 'MoveToStandbyPeg1',
                    'failure': 'ERROR'})
        smach.StateMachine.add('MoveToStandbyPeg1', collab_smach.MoveToFrameNode('wam', frame='location4'),
                transitions={
                    'success': 'MoveToRing',
                    'moveit_error': 'MoveToStandbyPeg1',
                    'ik_error': 'MoveToStandbyPeg1',
                    'failure': 'ERROR'})
        smach.StateMachine.add('MoveToRing', collab_smach.MoveToFrameNode('wam',frame='ring1/grasp2'),
                transitions={
                    'success': 'GrabRing',
                    'moveit_error': 'MoveToRing',
                    'ik_error': 'MoveToRing',
                    'failure': 'ERROR'})
        smach.StateMachine.add('GrabRing', collab_smach.CloseGripperNode('wam', attach='ring1'),
                transitions={
                    'success': 'LiftRing',
                    'failure': 'ERROR'})
        smach.StateMachine.add('LiftRing', collab_smach.MoveToFrameNodeIK('wam','lift_location', arm1_ik, arm1_stop),
                transitions={
                    'success': 'MoveRingToReachable',
                    'failure': 'ERROR'})
        smach.StateMachine.add('MoveRingToReachable', collab_smach.MoveToFrameNode('wam',frame='location1', objs=['ring1'], predicate=reachablePredicate),
                transitions={
                    'success': 'Arm2MoveToRing',
                    'moveit_error': 'MoveRingToReachable',
                    'ik_error': 'MoveRingToReachable',
                    'failure': 'ERROR'})
        smach.StateMachine.add('Arm2MoveToRing', collab_smach.MoveToFrameNode('wam2',frame='ring1/grasp1', predicate=grabRingPredicate),
                transitions={
                    'success': 'Arm2GrabRing',
                    'moveit_error': 'Arm2MoveToRing',
                    'ik_error': 'Arm2MoveToRing',
                    'failure': 'ERROR'})
        smach.StateMachine.add('Arm2GrabRing', collab_smach.CloseGripperNode('wam2', attach='ring1'),
                transitions={
                    'success': 'Arm1ReleaseRing',
                    'failure': 'ERROR'})
        smach.StateMachine.add('Arm1ReleaseRing', collab_smach.OpenGripperNode('wam', detach='ring1'),
                transitions={
                    'success': 'Arm1MoveBack',
                    'failure': 'ERROR'})
        smach.StateMachine.add('Arm1MoveBack', collab_smach.MoveToFrameNode('wam', frame='location4'),
                transitions={
                    'success': 'Arm2MoveToDrop',
                    'moveit_error': 'Arm1MoveBack',
                    'ik_error': 'Arm1MoveBack',
                    'failure': 'ERROR'})
        smach.StateMachine.add('Arm2MoveToDrop', collab_smach.MoveToFrameNode('wam2', frame='peg1/peg_link', objs=['ring','peg1']),
                transitions={
                    'success': 'Arm2Drop',
                    'moveit_error': 'Arm2MoveToDrop',
                    'ik_error': 'Arm2MoveToDrop',
                    'failure': 'ERROR'})
        smach.StateMachine.add('Arm2Drop', collab_smach.OpenGripperNode('wam2', detach='ring1'),
                transitions={
                    'success': 'DONE',
                    'failure': 'ERROR'})



    # Create SMACH introspection server
    sis = smach_ros.IntrospectionServer('peg_task_introspection_server', sm, '/SM_ROOT')
    sis.start()

    # wait for necessary services
    rospy.loginfo("TASK >>> waiting for services")
    rospy.wait_for_service("/predicator/get_assignment")

    # execute the state machine
    try:
        outcome = sm.execute()
    except smach.exceptions.InvalidUserCodeError, e:
        outcome = "ERROR"
        print e

    rospy.loginfo("TASK >>> Done state machine?")
    rospy.loginfo("Outcome = %s", outcome)

    sis.stop()

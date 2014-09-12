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
    grabRingPredicate.predicate = 'grasp_point_of'
    grabRingPredicate.num_params = 2
    grabRingPredicate.params[0] = '*'
    grabRingPredicate.params[1] = 'ring1'

    dropPointPredicate = PredicateStatement()
    dropPointPredicate.predicate = 'drop_point_of'
    dropPointPredicate.num_params = 2
    dropPointPredicate.params[0] = '*'
    dropPointPredicate.params[1] = 'peg2/base_link'

    releasePointPredicate = PredicateStatement()
    releasePointPredicate.predicate = 'release_point_of'
    releasePointPredicate.num_params = 2
    releasePointPredicate.params[0] = '*'
    releasePointPredicate.params[1] = 'ring1'

    invertDropPredicate = PredicateStatement()
    invertDropPredicate.predicate = "up_from"
    invertDropPredicate.num_params = 3
    invertDropPredicate.params[0] = "wam2/wrist_palm_link"
    invertDropPredicate.params[1] = "ring1/ring_link"
    invertDropPredicate.params[2] = "ring1/ring_link"

    pub = rospy.Publisher('/stop_aggregating_data', std_msgs.msg.Empty)

    with sm:
        smach.StateMachine.add('Open1', collab_smach.OpenGripperNode('wam'),
                transitions={
                    'success': 'Open2',
                    'failure': 'ERROR'})
        smach.StateMachine.add('Open2', collab_smach.OpenGripperNode('wam2'),
                transitions={
                    'success': 'MoveToStandbyPeg1',
                    'failure': 'ERROR'})
        #smach.StateMachine.add('ResetWam', collab_smach.ResetPoseNode('/gazebo/traj_rml/joint_traj_point_cmd'),
        #        transitions={
        #            'success': 'ResetWam2',
        #            'failure': 'ERROR'})
        #smach.StateMachine.add('ResetWam2', collab_smach.ResetPoseNode('/gazebo/w2traj_rml/joint_traj_point_cmd'),
        #        transitions={
        #            'success': 'WaitForReset',
        #            'failure': 'ERROR'})
        #smach.StateMachine.add('WaitForReset', collab_smach.TimedSleepNode(5.0),
        #        transitions={
        #            'success': 'MoveToStandbyPeg1'})
        smach.StateMachine.add('MoveToStandbyPeg1', collab_smach.MoveToFrameNode('wam', frame='peg1_standby_location'),
                transitions={
                    'success': 'MoveToRing',
                    'moveit_error': 'MoveToStandbyPeg1',
                    'ik_error': 'MoveToStandbyPeg1',
                    'no_predicates':'ERROR',
                    'failure': 'ERROR'})
        smach.StateMachine.add('MoveToRing', collab_smach.MoveToFrameNodeIK('wam','ring1/grasp2', arm1_ik, arm1_stop),
                transitions={
                    'success': 'GrabRing',
                    'failure': 'ERROR'})
        smach.StateMachine.add('GrabRing', collab_smach.CloseGripperNode('wam', attach='ring1'),
                transitions={
                    'success': 'LiftRing',
                    'failure': 'ERROR'})
        smach.StateMachine.add('LiftRing', collab_smach.MoveToFrameNodeIK('wam','peg1_lift_location', arm1_ik, arm1_stop),
                transitions={
                    'success': 'MoveRingToReachable',
                    'failure': 'ERROR'})
        smach.StateMachine.add('MoveRingToReachable', collab_smach.MoveToFrameNode('wam', objs=['ring1'], predicate=reachablePredicate),
                transitions={
                    'success': 'Arm2MoveToRing',
                    'moveit_error': 'MoveRingToReachable',
                    'ik_error': 'MoveRingToReachable',
                    'no_predicates': 'ReachableWait',
                    'failure': 'ERROR'})
        smach.StateMachine.add('ReachableWait', collab_smach.TimedSleepNode(5.0),
                transitions={'success': 'MoveRingToReachable'})
        smach.StateMachine.add('Arm2MoveToRing', collab_smach.MoveToFrameNode('wam2',frame='ring1/grasp1', predicate=grabRingPredicate),
                transitions={
                    'success': 'Arm2GrabRing',
                    'moveit_error': 'Arm2MoveToRing',
                    'ik_error': 'Arm2MoveToRing',
                    'no_predicates': 'Arm2MoveToRingWait',
                    'failure': 'ERROR'})
        smach.StateMachine.add('Arm2MoveToRingWait', collab_smach.TimedSleepNode(5.0),
                transitions={'success': 'Arm2MoveToRing'})
        smach.StateMachine.add('Arm2GrabRing', collab_smach.CloseGripperNode('wam2', attach='ring1'),
                transitions={
                    'success': 'Arm1ReleaseRing',
                    'failure': 'ERROR'})
        smach.StateMachine.add('Arm1ReleaseRing', collab_smach.OpenGripperNode('wam', detach='ring1'),
                transitions={
                    'success': 'Arm1MoveBack',
                    'failure': 'ERROR'})
        smach.StateMachine.add('Arm1MoveBack', collab_smach.MoveToFrameNode('wam', frame='peg1_standby_location'),
                transitions={
                    'success': 'Arm2MoveToPreDrop',
                    'moveit_error': 'Arm1MoveBackIK',
                    'ik_error': 'Arm1MoveBackIK',
                    'no_predicates':'ERROR',
                    'failure': 'ERROR'})
        smach.StateMachine.add('Arm1MoveBackIK', collab_smach.MoveToFrameNodeIK('wam', 'peg1_standby_location', arm1_ik, arm1_stop),
                transitions={
                    'success': 'Arm2MoveToPreDrop',
                    'failure': 'ERROR'})
        smach.StateMachine.add('Arm2MoveToPreDrop', collab_smach.MoveToFrameNode('wam2', frame='peg2_standby_location', objs=['ring1'], flip=True),
                transitions={
                    'success': 'Arm2MoveToDrop',
                    'moveit_error': 'Arm2MoveToPreDrop',
                    'ik_error': 'Arm2MoveToDrop',
                    'no_predicates': 'ERROR',
                    'failure': 'ERROR'})
        smach.StateMachine.add('Arm2MoveToDrop', collab_smach.MoveToFrameNode('wam2', objs=['ring1'], predicate=releasePointPredicate), #, with_offset=('wam2/wrist_palm_link','ring1/ring_link')),
                transitions={
                    'success': 'Arm2Drop',
                    'moveit_error': 'Arm2MoveToPreDrop',
                    'ik_error': 'Arm2MoveToDrop',
                    'no_predicates': 'Arm2MoveToDrop',
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

    pub.publish()

    sis.stop()

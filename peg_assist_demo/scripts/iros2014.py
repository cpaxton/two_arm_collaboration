#!/usr/bin/env python

import roslib; roslib.load_manifest('peg_assist_demo')
import rospy
import smach
import smach_ros

import collab_smach


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

    with sm:
        smach.StateMachine.add('Open1', collab_smach.OpenGripperNode('wam'),
                transitions={
                    'success': 'Open2',
                    'failure': 'ERROR'})
        smach.StateMachine.add('Open2', collab_smach.OpenGripperNode('wam2'),
                transitions={
                    'success': 'MoveToStandbyPeg1',
                    'failure': 'ERROR'})
        smach.StateMachine.add('MoveToStandbyPeg1', collab_smach.MoveToFrameNode('wam', 'location4'),
                transitions={
                    'success': 'MoveToRing',
                    'moveit_error': 'MoveToStandbyPeg1',
                    'failure': 'ERROR'})
        smach.StateMachine.add('MoveToRing', collab_smach.MoveToFrameNode('wam','ring1/grasp2'),
                transitions={
                    'success': 'GrabRing',
                    'moveit_error': 'MoveToRing',
                    'failure': 'ERROR'})
        smach.StateMachine.add('GrabRing', collab_smach.CloseGripperNode('wam', attach='ring1'),
                transitions={
                    'success': 'LiftRing',
                    'failure': 'ERROR'})
        smach.StateMachine.add('LiftRing', collab_smach.MoveToFrameNodeIK('wam','lift_location', arm1_ik, arm1_stop),
                transitions={
                    'success': 'MoveRingToReachable',
                    'failure': 'ERROR'})
        smach.StateMachine.add('MoveRingToReachable', collab_smach.MoveToFrameNode('wam','location1', objs=['ring1']),
                transitions={
                    'success': 'Arm2MoveToRing',
                    'moveit_error': 'MoveRingToReachable',
                    'failure': 'ERROR'})
        smach.StateMachine.add('Arm2MoveToRing', collab_smach.MoveToFrameNode('wam2','ring1/grasp1'),
                transitions={
                    'success': 'Arm2GrabRing',
                    'moveit_error': 'Arm2MoveToRing',
                    'failure': 'ERROR'})
        smach.StateMachine.add('Arm2GrabRing', collab_smach.CloseGripperNode('wam2'),
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

#!/usr/bin/env python

import rospy
import rostopic
from trajectory_msgs.msg import JointTrajectoryPoint
from oro_barrett_msgs.msg import BHandCmd

'''
TODO:
This script should contain startup code.
It tells the arms to take a starting position, for one.
'''

rospy.init_node('startup_node_arm1_peg_demo')

arm1_up = False

topic1 = "/gazebo/traj_rml/joint_traj_point_cmd"
gripper_topic1 = "/gazebo/barrett_manager/hand/cmd"

hand_closed = BHandCmd()

# set up the hand closed position
hand_closed.mode = [3, 3, 3, 3]
hand_closed.cmd = [2.0, 2.0, 2.0, 0]

try:

    pt = JointTrajectoryPoint()
    pt.positions = [0, -1.5707, 0, 3.1415, 0, -1.5705, 0]

    pub = rospy.Publisher(topic1, JointTrajectoryPoint)
    gpub = rospy.Publisher(gripper_topic1, BHandCmd)

    gpub.publish(hand_closed)

    while not arm1_up:
        try:

            #rostopic.get_info_text(topic1, blocking=True)
            (tp, name, c) = rostopic.get_topic_type(topic1, blocking=True)
            if not name == None:
                arm1_up = True

        except: continue

    print pt

    for i in range(1,20):
        rospy.loginfo("trying to publish point for arm1")
        pub.publish(pt)
        gpub.publish(hand_closed)
        rospy.sleep(0.5)

except rospy.ROSInterruptException, e: pass

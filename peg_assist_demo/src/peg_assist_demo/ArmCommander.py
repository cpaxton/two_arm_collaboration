#!/usr/bin/env python
import roslib
import rospy
import rosbag # reading bag files

from geometry_msgs.msg import TransformStamped
from cartesian_trajectory_msgs.msg import CartesianTrajectory
from cartesian_trajectory_msgs.msg import CartesianTrajectoryPoint
from oro_barrett_msgs.msg import BHandCmd

'''
ArmCommander.py

This file contains code to republish cartesian trajectories for the WAM arms.
It reads from a topic like 'wam2/cmd_traj' and republishes 'wam2/cmd'.

It will continually publish a 'home' position at first, and the last sent position after that.

'''

class ArmCommander(object):
    def __init__(self):
        self.home = True
        self.transform = TransformStamped()
        self.hand_topic = rospy.getParam("hand_topic","/barrett/hand/cmd")
        self.hand_opened = BHandCmd()
        self.hand_closed = BHandCmd()
        
        # set up the hand open position
        self.hand_opened.mode = [3, 3, 3, 3]
        self.hand_opened.cmd = [0.5, 0.5, 0.5, 0]

        # set up the hand closed position
        self.hand_closed.mode = [3, 3, 3, 3]
        self.hand_closed.cmd = [2.0, 2.0, 2.0, 2.0]
        
        self.hand_pub = rospy.Publisher(self.hand_topic, BHandCmd, 10)

    '''
    reset_cb()
    set published transform back to home position
    '''
    def reset_cb(self) :
        self.home = True

    def close_cb(self) :
        self.closed = True

    def open_cb(self) :
        self.closed = False

    def traj_cb(self) :
        self.home = False

        # iterate over trajectory

if __name__ == '__main__' :

    rospy.init_node('arm_command_node')



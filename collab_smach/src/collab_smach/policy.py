# Python file for policy-based controllers
# These use weighted data from predicator to move around in joint space, and are the basis for our low-level controllers
# This includes 

import rospy
import roslib; roslib.load_manifest("collab_smach")
import smach
import smach_ros
import actionlib
import tf

import numpy as np


'''

Dimensions:
    7 degrees of freedom = 7 dimensions
    + gripper open/closed?

Variables:
    distance to collision
    distance to waypoints/grasp points
    relative position
'''
class PolicyMoveNode(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['success','failure']

    def execute(self, userdata):
        pass

'''

We have another node which moves based on a predicate.
Performs some move until a predicate is met.
'''
class PredicateMoveNode(smach.State):
    def __init__(self, robot, predicate):
        smach.State.__init__(self, outcomes=['success','failure']

    def execute(self, userdata):
        pass

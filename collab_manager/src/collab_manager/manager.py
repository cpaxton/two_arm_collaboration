#!/usr/bin/env python

import rospy

from oro_barrett_msgs.msg import *
from collab_msgs.srv import *
from collab_msgs.msg import *
import tf
import actionlib

'''
Publish the current gripper/IK commands to the arms
'''
class CollabManager(object):

    def __init__(self, arms=2):

        self.config_ = rospy.get_param('~topic_config')

        if(arms > 2):
            print "More than two arms not supported at this time."

        if (arms == 2):
            # start up publishers and subscribers for arm 2
            pass

        # start up publishers and subscribers for arm 1
        self.close_server = actionlib.SimpleActionServer('collaboration/close',
                StoredAction,
                self.close_grippers,False)
        self.open_server = actionlib.SimpleActionServer('collaboration/open',
                StoredAction,
                self.open_grippers, False)
        self.move_server = actionlib.SimpleActionServer('collaboration/move_to_destination',
                StoredAction,
                self.move_to_destination, False)

    '''
    tick()
    publish command messages for the 
    '''
    def tick(self):
        pass

    def close_grippers(self, goal):
        pass

    def open_grippers(self, goal):
        pass

    def move_to_destination(self, goal):
        pass

if __name__ == "__main__":

    rospy.init_node('collaboration_arm_manager')

    spin_rate = rospy.get_param('rate',10)
    arms = int(rospy.get_param('arms', 2))
    rate = rospy.Rate(spin_rate)

    print "starting multi-arm collaboration manager"

    try:

        cm = CollabManager(arms)

        while not rospy.is_shutdown():

            rate.sleep()

    except rospy.ROSInterruptException: pass

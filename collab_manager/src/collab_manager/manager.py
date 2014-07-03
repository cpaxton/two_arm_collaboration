#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest('collab_msgs')

from oro_barrett_msgs.msg import BHandCmd
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
        self.hand_opened = BHandCmd()
        self.hand_closed = BHandCmd()
        
        # set up the hand open position
        self.hand_opened.mode = [3, 3, 3, 3]
        self.hand_opened.cmd = [0.5, 0.5, 0.5, 0]

        # set up the hand closed position
        self.hand_closed.mode = [3, 3, 3, 3]
        self.hand_closed.cmd = [2.0, 2.0, 2.0, 2.0]

        if(arms > 2):
            print "More than two arms not supported at this time."

        if (arms == 2):
            # start up publishers and subscribers for arm 2
            pass

        # start up publishers and subscribers for arm 1
        self.close_server = actionlib.SimpleActionServer('collab/close',
                StoredTaskAction,
                self.close_grippers, False)
        self.open_server = actionlib.SimpleActionServer('collab/open',
                StoredTaskAction,
                self.open_grippers, False)
        self.move_server = actionlib.SimpleActionServer('collab/move_to_destination',
                StoredTaskAction,
                self.move_to_destination, False)

        self.close_server.start()
        self.open_server.start()
        self.move_server.start()

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

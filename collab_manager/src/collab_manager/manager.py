#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest('collab_msgs')
import math

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

        self.id_ = self.config_['id']
        self.gripper_topic = self.config_['gripper_cmd']
        self.ik_topic = self.config_['ik_cmd']
        self.gripper_pub = rospy.Publisher(self.config_['gripper_cmd'], BHandCmd)
        self.base_link = self.config_['base_link']

        self.hand_opened = BHandCmd()
        self.hand_closed = BHandCmd()
        
        # set up the hand open position
        self.hand_opened.mode = [3, 3, 3, 3]
        self.hand_opened.cmd = [0.5, 0.5, 0.5, 0]

        # set up the hand closed position
        self.hand_closed.mode = [3, 3, 3, 3]
        self.hand_closed.cmd = [2.0, 2.0, 2.0, 2.0]

        # what frame should we be following?
        self.destination_frame = ''
        self.ik = ((-0.50, 0, 0.8), tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0))
        
        # what is the present state of the gripper?
        self.gripper_state = 0

        # start up publishers and subscribers for arm 1
        self.close_server = actionlib.SimpleActionServer('collab/' + self.id_ + '/close',
                StoredTaskAction,
                self.close_grippers, False)
        self.open_server = actionlib.SimpleActionServer('collab/' + self.id_ + '/open',
                StoredTaskAction,
                self.open_grippers, False)
        self.move_server = actionlib.SimpleActionServer('collab/' + self.id_ + '/move_to_destination',
                StoredTaskAction,
                self.move_to_destination, False)

        self.close_server.start()
        self.open_server.start()
        self.move_server.start()

    '''
    send_default_transform()
    helper function that 
    '''
    def send_default_transform(self):
        br.sendTransform((-0.50, 0, 0.8), 
                tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
                rospy.Time.now(),
                self.ik_topic,
                self.base_link)

    '''
    tick()
    publish command messages for the arms
    '''
    def tick(self):
        br = tf.TransformBroadcaster()

        if self.gripper_state == 0:
            self.gripper_pub.publish(self.hand_closed)
        else:
            self.gripper_pub.publish(self.hand_opened)


        if len(self.destination_frame) == 0:
            # publish default transform
            self.send_default_transform()
        else:
            # publish a new transform leading to whatever location
            (trans, rot) = self.ik
            br.sendTransform(trans, rot, rospy.Time.now(),
                self.ik_topic,
                self.base_link)
                

    def close_grippers(self, goal):
        self.gripper_state = self.hand_closed
        
        while (rospy.Time.now() - start).to_sec() < goal.secs:
            pass
            # check for is_closed

        # wait for gripper to close
        res = StoredTaskActionResult()

    def open_grippers(self, goal):
        self.gripper_state = self.hand_opened

        while (rospy.Time.now() - start).to_sec() < goal.secs:
            pass
            # check for is_closed


        # wait for gripper to open
        res = StoredTaskActionResult()
        return res

    def move_to_destination(self, goal):
        start = rospy.Time.now()

        res = StoredTaskActionResult()
        feedback = StoredTaskActionFeedback()

        feedback.step = StoredTaskActionFeedback.RETRIEVING_PARAMS

        while (rospy.Time.now() - start).to_sec() < goal.secs:

            feedback.step = StoredTaskActionFeedback.MOVING
            self.move_server.publish_feedback(feedback)

            try:
                pass

                # get the transform
                # publish it as wam/cmd or whatever

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        
        # if it didn't finish, error
        res.result.info = "FAILED"

        return res
        

if __name__ == "__main__":

    rospy.init_node('collaboration_arm_manager')

    spin_rate = rospy.get_param('rate',10)
    arms = int(rospy.get_param('arms', 2))
    rate = rospy.Rate(spin_rate)

    print "starting multi-arm collaboration manager"

    try:

        cm = CollabManager(arms)

        while not rospy.is_shutdown():
            cm.tick()
            rate.sleep()

    except rospy.ROSInterruptException: pass

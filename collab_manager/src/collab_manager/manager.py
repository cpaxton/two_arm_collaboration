#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest('collab_msgs')
import math

from oro_barrett_msgs.msg import BHandCmd
from collab_msgs.srv import *
from collab_msgs.msg import *
import tf
import actionlib

from predicator_core.srv import *

'''
Publish the current gripper/IK commands to the arms
'''
class CollabManager(object):

    def __init__(self):

        self.config_ = rospy.get_param('~topic_config')
        self.world_frame = rospy.get_param('~world_frame','/world')

        self.id_ = self.config_['id']
        self.gripper_topic = self.config_['gripper_cmd']
        self.ik_topic = self.config_['ik_cmd']
        self.gripper_pub = rospy.Publisher(self.config_['gripper_cmd'], BHandCmd)
        self.base_link = self.config_['base_link']

        print "starting multi-arm collaboration manager for arm \"%s\""%(self.id_)

        self.test_predicate = rospy.ServiceProxy('predicator/test_predicate', TestPredicate)

        self.hand_opened = BHandCmd()
        self.hand_closed = BHandCmd()
        
        # set up the hand open position
        self.hand_opened.mode = [3, 3, 3, 3]
        self.hand_opened.cmd = [0.5, 0.5, 0.5, 0]

        # set up the hand closed position
        self.hand_closed.mode = [3, 3, 3, 3]
        self.hand_closed.cmd = [2.0, 2.0, 2.0, 0]

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

        self.br = tf.TransformBroadcaster()

    '''
    send_default_transform()
    helper function that 
    '''
    def send_default_transform(self):
        self.br.sendTransform((-0.50, 0, 0.8), 
                tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
                rospy.Time.now(),
                self.ik_topic,
                self.base_link)

    '''
    tick()
    publish command messages for the arms
    tick is in a separate thread from the actions that actually change these things
    '''
    def tick(self):

        if self.gripper_state == 0:
            self.gripper_pub.publish(self.hand_closed)
        else:
            self.gripper_pub.publish(self.hand_opened)


        #if len(self.destination_frame) == 0:
        #    # publish default transform
        #    self.send_default_transform()
        #else:
        # publish a new transform leading to whatever location
        (trans, rot) = self.ik
        self.br.sendTransform(trans, rot, rospy.Time.now(),
            self.ik_topic,
            self.base_link)
                
    '''
    close_grippers()
    publish a message telling the grippers to close
    check to see if it closed
    '''
    def close_grippers(self, goal):
        self.gripper_state = self.hand_closed

        res = StoredTaskResult()
        feedback = StoredTaskFeedback()

        feedback.update.step = Step.STARTING
        self.close_server.publish_feedback(feedback)

        while (rospy.Time.now() - start).to_sec() < goal.secs:

            feedback.update.step = Step.MOVING
            self.close_server.publish_feedback(feedback)
            break # remove this when we have something to check the condition
            # check for is_closed

        # wait for gripper to close
        res = StoredTaskResult()
        self.close_server.set_succeeded()

        return res

    '''
    open_grippers()
    change gripper_state to open
    check to see if it opened
    '''
    def open_grippers(self, goal):
        self.gripper_state = self.hand_opened

        res = StoredTaskResult()
        feedback = StoredTaskFeedback()

        feedback.update.step = Step.STARTING
        self.open_server.publish_feedback(feedback)

        while (rospy.Time.now() - start).to_sec() < goal.secs:

            feedback.update.step = Step.MOVING
            self.open_server.publish_feedback(feedback)
            break # remove this when we have something to check the condition
            # check for is_closed


        # wait for gripper to open
        res = StoredTaskResult()
        self.close_server.set_succeeded()

        return res

    def move_to_destination(self, goal):
        start = rospy.Time.now()

        res = StoredTaskResult()
        feedback = StoredTaskFeedback()

        feedback.update.step = Step.STARTING
        self.move_server.publish_feedback(feedback)

        listener = tf.TransformListener()
        found_transform = False

        while (rospy.Time.now() - start).to_sec() < goal.secs:

            feedback.update.step = Step.RETRIEVING_PARAMS
            self.move_server.publish_feedback(feedback)

            if found_transform == False:
                try:

                    # get the transform
                    self.ik = listener.lookupTransform(self.base_link, goal.id, rospy.Time.now())
                    found_transform = True
                    break # remove this when we have something to check the condition

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
        
        # if it didn't finish, error
        # else, success
        if found_transform == True:
            res.result.info = "Set TF frame for movement"
            res.result.final = Step.FINISHED_SUCCESS
            self.move_server.set_succeeded()
        else:
            res.result.info = "Failed to look up transform!"
            res.result.final = Step.RETRIEVING_PARAMS
            self.move_server.set_aborted()
        
        return res

if __name__ == "__main__":

    rospy.init_node('collaboration_arm_manager')

    spin_rate = rospy.get_param('rate',10)
    rate = rospy.Rate(spin_rate)

    try:

        cm = CollabManager()

        while not rospy.is_shutdown():
            cm.tick()
            rate.sleep()

    except rospy.ROSInterruptException:
        cm.close_server.need_to_terminate = True
        cm.open_server.need_to_terminate = True
        cm.move_server.need_to_terminate = True


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

        self.gripper_topics = {}
        self.ik_topics = {}
        self.gripper_pubs = {}
        self.ik_pubs = {}

        self.grippers = {}
        self.ik = {}

        self.robots = []

        self.bases = {}

        for elem in self.config_:

            robot = elem['id']

            self.robots.append(robot)
            self.gripper_topics[robot] = elem['gripper_cmd']
            self.ik_topics[robot] = elem['ik_cmd']
            self.gripper_pubs[robot] = rospy.Publisher(elem['gripper_cmd'], BHandCmd)
            self.bases[robot] = elem['base_link']

        print self.gripper_topics
        print self.ik_topics

        self.hand_opened = BHandCmd()
        self.hand_closed = BHandCmd()
        
        # set up the hand open position
        self.hand_opened.mode = [3, 3, 3, 3]
        self.hand_opened.cmd = [0.5, 0.5, 0.5, 0]

        # set up the hand closed position
        self.hand_closed.mode = [3, 3, 3, 3]
        self.hand_closed.cmd = [2.0, 2.0, 2.0, 2.0]

        #self.home = {}
        #for robot in self.robots:
        #    home[robot] = ((0.8, 0.46, 1.0), 

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
    publish command messages for the arms
    '''
    def tick(self):
        br = tf.TransformBroadcaster()
        for robot in self.robots:
            if robot in self.grippers:
                if self.grippers[robot] == 0:
                    self.gripper_pubs.publish(self.hand_closed)
                else:
                    self.gripper_pubs.publish(self.hand_opened)
            if robot in self.ik:
                (trans, rot) = self.ik[robot]
                br.sendTransform(trans, rot, rospy.Time.now(),
                        self.ik_topics[robot],
                        self.bases[robot])
            else:
                # publish default transform
                br.sendTransform((-0.50, 0, 0.8), 
                        tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
                        rospy.Time.now(),
                        self.ik_topics[robot],
                        self.bases[robot])
                

    def close_grippers(self, goal):
        self.grippers[goal.id] = self.hand_closed
        
        while (rospy.Time.now() - start).to_sec() < goal.secs:
            pass

        # wait for gripper to close
        res = StoredTaskActionResult()

    def open_grippers(self, goal):
        self.grippers[goal.id] = self.hand_opened

        while (rospy.Time.now() - start).to_sec() < goal.secs:
            pass


        # wait for gripper to open
        res = StoredTaskActionResult()

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

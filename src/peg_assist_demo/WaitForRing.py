#!/usr/bin/env python

import roslib; roslib.load_manifest('peg_assist_demo')
import rospy
import actionlib

import * from peg_assist_demo.msg

class WaitForRingAction(object) :
    # create messages
    _feedback = peg_assist_demo.msg.WaitForRingFeedback()
    _result = peg_assist_demo.msg.WaitForRingResult()

    def __init__(self, name) :
        self._action_name = "WaitForRing"
        self._as = actionlib.SimpleActionServer("WaitForRing",
                peg_assist_demo.msg.WaitForRingAction,
                execute_cb = self.execute_cb,
                auto_start = False)
        self._as.start()

    def execute_cb(self, goal) :
        # helper variables
        r = rospy.Rate(1)
        success = True



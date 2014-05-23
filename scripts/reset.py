#!/usr/bin/env python
import roslib;
roslib.load_manifest('lcsr_replay')

import rospy
import numpy as np

from replay import *

if __name__ == '__main__':
    rospy.init_node('replay_player_node')

    io = default_io_startup()
    io.parse()

    mode = rospy.get_param('~mode', 'segment');
    segment = rospy.get_param('~segment', -1)
    topic = rospy.get_param('~topic', '/wam/cmd')
    dest_topic = rospy.get_param('~dest_topic', topic)

    # send only the first point if that's what we want to do
    if mode == 'segment' :
        # get a trajectory
        (t, traj) = io.getTrajectory(topic, segment)
        io.publish(traj[0], 0, topic)
    # otherwise, send some arbitrary point
    else :
        # load something I guess
        print "Mode not implemented yet"

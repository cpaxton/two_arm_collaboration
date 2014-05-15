#!/usr/bin/env python
import roslib
import rospy
import rosbag # reading bag files

from lcsr_replay.msg import *

'''
REPLAY.PY

Methods to read, look over ROSBAG files with REPLAY add-on messages.
This includes features, segments, etc.

'''

class ReplayIO:
    bagfile = 'demo.bag'
    io_topics = ['wam/cmd', 'wam2/cmd', '/FEATURES', '/SEGMENT']

    '''
    parse()
    Takes a filename, opens the bag up.
    Should also parse out necessary information by segment.
    '''
    def parse(filename) :
        bagfile = filename


    '''
    setFilename()
    Takes filename, saves it to this replay object.
    '''
    def setFilename(filename) :
        bagfile = filename

    '''
    print()
    Display information about what was actually in the bag, for debugging.
    '''
    def print() :
        bag = rosbag.Bag(bagfile)
        for topic, msg, t in bag.read_messages(io_topics) :
            print "%f: %s"%(t, topic)


if __name__ == '__main__':
    rospy.init_node('replay_read_node')


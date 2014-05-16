#!/usr/bin/env python
import roslib
import rospy
import rosbag # reading bag files

from lcsr_replay.msg import *
from dmp.msg import *
from dmp.srv import *

'''
REPLAY.PY

Methods to read, look over ROSBAG files with REPLAY add-on messages.
This includes features, segments, etc.

'''

class ReplayIO:
    bagfile = 'demo.bag'
    io_topics = ['wam/cmd', 'wam2/cmd', '/FEATURES', '/SEGMENT']

    segment = []
    data = {}
    parsed = False

    '''
    parse()
    Takes a filename, opens the bag up.
    Should also parse out necessary information by segment.
    '''
    def parse(filename = bagfile) :
        bagfile = filename

        segment = []
        data = {}
        for topic in io_topics:
            data[topic] = []

        for topic, msg, t in bag.read_messages(io_topics) :

            if topic == "/SEGMENT" :
                data[topic] += [msg.num]
            else if topic == "/FEATURES" :
                print msg

            # TODO: convert format to 
            data{topic} += [msg]

        parsed = True


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

    def getTrajectory(topic) :

        if parsed == False :
            parse()

        bag = rosbag.Bag(bagfile)
           
        return data[topic]

'''
default_io_startup()
Default startup for the Replay package.
Reads parameter from command line and sets up an IO object.
'''
def default_io_startup():

    bagfile = rospy.getParam('bag', 'demo.bag')

    io = ReplayIO()
    io.setFilename(bagfile)

if __name__ == '__main__':
    rospy.init_node('replay_read_node')

    io = default_io_startup()



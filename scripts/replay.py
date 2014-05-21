#!/usr/bin/env python
import roslib
import rospy
import rosbag # reading bag files

from oro_barrett_msgs import *
from lcsr_replay.msg import *
from geometry_msgs.msg import TransformStamped

'''
REPLAY.PY

Methods to read, look over ROSBAG files with REPLAY add-on messages.
This includes features, segments, etc.

'''

'''
verbosity
Controls amount of text output produced by the script
'''
verbosity = 0

SEGMENT = '/SEGMENT'
FEATURES = '/FEATURES'

class ReplayEvent:
    def __init__(self) :
        self.arm1 = []
        self.arm2 = []
        self.hand1 = []
        self.hand2 = []
        self.features = []

'''
BhandToVector()
Convert a Bhand command message into a vector
'''
def BhandToVector(bhand_cmd_msg) :
    vec = [bhand_cmd_msg.mode[0], bhand_cmd_msg.mode[1],
            bhand_cmd_msg.mode[2], bhand_cmd_msg.mode[3],
            bhand_cmd_msg.cmd[0], bhand_cmd_msg.cmd[1],
            bhand_cmd_msg.cmd[2], bhand_cmd_msg.cmd[3]]
    return vec

'''
VectorToBhand()
Convert a vector back into a Barrett Hand command message
'''
def VectorToBhand(vector) :
    bhand_cmd_msg.mode = [vector[i] for i in range(0,4)]
    bhand_cmd_msg.cmd = [vector[i] for i in range(4,8)]
    return bhand_cmd_msg

def TfToVector(tform) :
    pos = [tform.translation.x, tform.translation.y, tform.translation.z]
    rot = [tform.rotation.x, tform.rotation.y, tform.rotation.z, tform.rotation.w]

    return [pos + rot]

def VectorToStampedTf(vector) :
    tform = TransformStamped()
    tf.transform.translation.x = vector[0];
    tf.transform.translation.y = vector[1];
    tf.transform.translation.z = vector[2];
    tf.transform.rotation.x = vector[3];
    tf.transform.rotation.y = vector[4];
    tf.transform.rotation.z = vector[5];
    tf.transform.rotation.w = vector[6];

    return tf

class ReplayIO:

    def __init__(self) :
        self.bagfile = 'demo.bag'
        self.io_topics = ['/wam/cmd', '/wam2/cmd', '/FEATURES', '/SEGMENT']
        self.trajectory_topics = ['/wam/cmd','/wam2/cmd']
        self.hand_topics = ['/hand/cmd', '/hand2/cmd']
        self.traj_pubs = []
        self.hand_pubs = []
        self.pubs = {}

        self.segment = []
        self.data = {}
        self.parsed = False

    '''
    addTrajectoryPublisher()
    Adds a single publisher, sending out transform messages.
    '''
    def addTrajectoryPublisher(self, topic) :
        if not topic in self.pubs :
            self.pubs[topic] = rospy.Publisher(topic, TransformStamped)
            self.traj_pubs += [topic]

    '''
    addHandPublisher()
    Adds a single publisher, sending out barrett hand messages.
    '''
    def addHandPublisher(self, topic) :
        if not topic in self.pubs :
            self.pubs[topic] = rospy.Publisher(topic, BHandCmd)
            self.hand_pubs += [topic]

    '''
    publish()
    Wait to time t and publish a message ("point") on topic
    '''
    def publish(self, point, t, topic) :
        rospy.sleep(t)
        if type(point) is list:
            if topic in self.traj_pubs:
                msg = VectorToTfStamped(point)
            elif topic in self.hand_pubs:
                msg = VectorToBhand(point)
            else :
                raise NameError("Unknown topic given list type!")
            self.pubs[topic].publish(msg)
        elif type(point) is BhandCmd and topic in self.hand_pubs:
            self.pubs[topic].publish(point)
        elif type(point) is TransformStamped and topic in self.traj_pubs:
            self.pubs[topic].publish(point)
        else :
            raise TypeError("Unknown topic or type!")

    '''
    play_trajectory()
    Runs through an entire trajectory given by something like DMP.
    Converts elements into message types and sends them by iteratively calling "publish()"
    '''
    def play_trajectory(traj, times, topic) :
        first_t = times[0]
        for point, t in zip(traj, times) :
            self.publish(point, t - first_t, topic)

    '''
    parse()
    Takes a filename, opens the bag up.
    Should also parse out necessary information by segment.
    '''
    def parse(self) :

        self.times = []
        self.data = {}
        for topic in self.io_topics:
            self.data[topic] = []

        bag = rosbag.Bag(self.bagfile)
        for topic, msg, t in bag.read_messages(self.io_topics) :

            if topic == SEGMENT :
                self.data[topic] += [msg.num]
                self.times += [t]
            elif topic == FEATURES :
                # loop over names and transforms in FEATURES msg
                for i in range(len(msg.names)) :
                    name = topic + msg.names[i]
                    if not name in self.data :
                        self.data[name] = []
                    self.data[name] += TfToVector(msg.transform[i])
            elif topic in self.trajectory_topics:
                # parse in the translation and rotation of the transform
                self.data[topic] += TfToVector(msg.transform)
            else :
                self.data[topic] += [msg]

            if verbosity > 0 and len(self.data[topic]) > 0:
                print "%f: %s=%s"%(t.to_sec(), topic, self.data[topic][-1])

        parsed = True



    '''
    setFilename()
    Takes filename, saves it to this replay object.
    '''
    def setFilename(self, filename) :
        self.bagfile = filename

    '''
    print()
    Display information about what was actually in the bag, for debugging.
    '''
    def printFile(self) :
        bag = rosbag.Bag(self.bagfile)
        for topic, msg, t in bag.read_messages(self.io_topics) :
            print "%f: %s"%(t.to_sec(), topic)

    '''
    getTrajectory()
    This should return the trajectory for a certain topic (and optionally segment)
    '''
    def getTrajectory(self, topic, segment=-1) :

        if self.parsed == False :
            self.parse()


        if segment >= 0 :
            idx = [i for (i, s) in zip(range(len(self.data[SEGMENT])), self.data[SEGMENT]) if s == segment]
        else :
            idx = range(len(self.data[topic]))

        if verbosity > 1:
            print idx
        
        if verbosity > 1:
            for i in idx :
                print [self.data[SEGMENT][i], self.data[topic][i]]

        return ([self.times[i].to_sec() for i in idx], [self.data[topic][i] for i in idx])


'''
default_io_startup()
Default startup for the Replay package.
Reads parameter from command line and sets up an IO object.
'''
def default_io_startup():

    bagfile = rospy.get_param('~bag', 'demo.bag')
    v = rospy.get_param('verbosity', 1)

    io = ReplayIO()
    io.setFilename(bagfile)

    # choose which topics we want to save here
    io.io_topics = ['/wam/cmd', '/wam2/cmd',
            '/gazebo/barrett_manager/hand/cmd', '/gazebo/w2barrett_manager/hand/cmd',
            '/SEGMENT', '/FEATURES']
    io.trajectory_topics = ['/wam/cmd','/wam2/cmd']
    io.hand_topics = ['/gazebo/barrett_manager/hand/cmd', '/gazebo/w2barrett_manager/hand/cmd']

    return io

if __name__ == '__main__':
    rospy.init_node('replay_read_node')

    print rospy.get_param('~bag')

    io = default_io_startup()
    io.printFile()
    io.parse()

    verbosity = 2

    print "Testing trajectory retrieval..."
    print io.getTrajectory('/wam/cmd', -1)


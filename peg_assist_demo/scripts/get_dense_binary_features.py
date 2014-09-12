#!/usr/bin/env python

import rospy
import rosbag

from collab_msgs.msg import SegmentLabel
from collab_msgs.msg import BinaryFeatures
from predicator_msgs.msg import PredicateList

import predicator_core
import std_msgs

import threading

X = [] # predicate features
Y = [] # segment labels

stop = False
segment = {}
values = {}
verbose = False
#next_id = 1
#current_id = 0
#instance = 0
#ids = {}

lock = threading.Lock()

seg = None

'''
update_last_segment()
Gets the last segment information and stores it
'''
def update_last_segment(msg):
    lock.acquire()
    seg = msg
    segment["name"] = msg.segment_name
    segment["instance"] = msg.segment_id
    lock.release()
    #instance = msg.segment_id
    #if msg.segment_name in ids:
    #    current_id = ids[msg.segment_name]
    #else:
    #    ids[msg.segment_name] = next_id
    #    next_id = next_id + 1

'''
Go through the list and check for predicates we are interested in
If they exist, add a 1; else add a 0
'''
def list_cb(msg):

    lock.acquire()
    for frame1 in geo_frames:
        for frame2 in geo_frames:
            if frame1 == frame2:
                continue
            for ref in geo_ref_frames:
                for pred in predicates['geometry3']:
                    key = predicator_core.get_key(pred, [frame1, frame2, ref])
                    values[key] = 0

    for frame1 in geo_frames:
        for frame2 in geo_frames:
            if frame1 == frame2:
                continue
            for pred in predicates['geometry2']:
                key = predicator_core.get_key(pred, [frame1, frame2, ''])
                values[key] = 0

    for frame1 in mv_frames:
        for frame2 in mv_frames:
            if frame1 == frame2:
                continue
            for pred in predicates['movement2']:
                key = predicator_core.get_key(pred, [frame1, frame2, ''])
                values[key] = 0

    for frame1 in mv_frames:
        for pred in predicates['movement1']:
            key = predicator_core.get_key(pred, [frame1, '', ''])
            values[key] = 0

    for frame1 in collision_from:
        for frame2 in collision_to:
            for pred in predicates['collision']:
                key = predicator_core.get_key(pred, [frame1, frame2, ''])
                values[key] = 0

    for param in joints:
        for pred in predicates['joints']:
            key = predicator_core.get_key(pred, [param, '', ''])
            values[key] = 0

    for item in msg.statements:
        key = predicator_core.get_key(item.predicate, item.params)
        if key in values:
            values[key] = 1
        elif verbose:
            print "WARNING: %s not in list!"%(key)

    if verbose:
        print len(values)
        for k, v in values.iteritems():
            print k, v
    lock.release()

def stop_cb(msg):
    global stop
    print "Stopping bagging!"
    stop = True

if __name__ == '__main__':

    rospy.init_node('get_dense_binary_features_node')
    segment_topic = rospy.get_param('~segment_topic', '/segment_labels')
    predicate_topic = rospy.get_param('~predicate_topic', '/predicator/list')
    stop_topic = rospy.get_param('~stop_topic', '/stop_aggregating_data')

    output_topic = rospy.get_param('~output_topic', '/binary_features'

    segment_sub = rospy.Subscriber(segment_topic, SegmentLabel, update_last_segment)
    list_sub = rospy.Subscriber(predicate_topic, PredicateList, list_cb)
    pub = rospy.Publisher(output_topic, BinaryFeatures)
    stop_sub = rospy.Subscriber(stop_topic, std_msgs.msg.Empty, stop_cb)
    
    geo_frames = rospy.get_param("~geometry_frames")
    geo_ref_frames = rospy.get_param("~geometry_reference_frames")
    mv_frames = rospy.get_param("~movement_frames")
    collision_from = rospy.get_param("~collision_from")
    collision_to = rospy.get_param("~collision_to")
    predicates = rospy.get_param("~predicates")
    joints = rospy.get_param("~joints_params")
    save_param_names = rospy.get_param("~save_param_names","true") == "true"

    bagfile = rospy.get_param("~bagfile", "")
    print bagfile

    save = True
    if bagfile == "":
        print "Not saving to a bag."
        save = False
    else:
        print "Saving to file."

    rate = rospy.Rate(5)

    try:

        if save:
            bag = rosbag.Bag(bagfile, 'w')
            print "File opened."

        while not rospy.is_shutdown() and not stop:
            lock.acquire()
            if "name" in segment:
                name = segment["name"]
                instance = segment["instance"]
                print name, instance, len(values)

                msg = BinaryFeatures()
                msg.segment = SegmentLabel(name, instance)
                if save_param_names:
                    msg.name = [k for k in values.iterkeys()]
                msg.value = [v for v in values.itervalues()]

                if(verbose):
                    print msg

                pub.publish(msg)

                if save:
                    bag.write(output_topic, msg)

            lock.release()
            rate.sleep()



    except rospy.ROSInterruptException, e:
        if save:
            bag.close()
        else:
            pass

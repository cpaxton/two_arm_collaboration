#!/usr/bin/env python

import rospy
import rosbag

from collab_msgs.msg import SegmentLabel
from predicator_msgs.msg import PredicateList

import predicator_core

X = [] # predicate features
Y = [] # segment labels

values = {}
verbose = False

'''
update_last_segment()
Gets the last segment information and stores it
'''
def update_last_segment(msg):
    seg = msg

'''
Go through the list and check for predicates we are interested in
If they exist, add a 1; else add a 0
'''
def list_cb(msg):

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

    for item in msg.statements:
        key = predicator_core.get_key(item.predicate, item.params)
        if key in values:
            values[key] = 1
        elif verbose:
            print "WARNING: %s not in list!"%(key)

    print len(values)


if __name__ == '__main__':

    rospy.init_node('get_dense_binary_features_node')
    segment_topic = rospy.get_param('~segment_topic', '/segment_labels')
    predicate_topic = rospy.get_param('~predicate_topic', '/predicator/list')

    segment_sub = rospy.Subscriber(segment_topic, SegmentLabel, update_last_segment)
    list_sub = rospy.Subscriber(predicate_topic, PredicateList, list_cb)
    
    geo_frames = rospy.get_param("~geometry_frames")
    geo_ref_frames = rospy.get_param("~geometry_reference_frames")
    mv_frames = rospy.get_param("~movement_frames")
    collision_from = rospy.get_param("~collision_from")
    collision_to = rospy.get_param("~collision_to")
    predicates = rospy.get_param("~predicates")

    rospy.spin()

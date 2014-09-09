#!/usr/bin/env python

import rospy
import rosbag

from collab_msgs.msg import SegmentLabel
from predicator_msgs.msg import PredicateList

import predicator_core

X = [] # predicate features
Y = [] # segment labels

values = {}

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
    for item in msg.statements:
        values[predicator_core.get_key(item.predicate, item.params)] = 0


if __name__ == '__main__':

    rospy.init_node('get_dense_binary_features_node')
    segment_topic = rospy.get_param('~segment_topic', '/segment_labels')
    predicate_topic = rospy.get_param('~predicate_topic', '/predicator/list')

    segment_sub = rospy.Subscriber(segment_topic, SegmentLabel, update_last_segment)
    list_sub = rospy.Subscriber(predicate_topic, PredicateList, list_cb)
    
    geo_frames = rospy.get_param("~geometry_frames")
    geo_ref_frames = rospy.get_param("~geometry_reference_frames")
    mv_frames = rospy.get_param("~movement_frames")
    predicates = rospy.get_param("~predicates")

    print predicates['geometry3']

    rospy.spin()

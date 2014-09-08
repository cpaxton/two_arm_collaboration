#!/usr/bin/env python

import rospy

from smach_msgs.msg import SmachContainerStatus
from collab_msgs.msg import SegmentLabel

'''
read from a rosbag
get labels for different segments
tell us when a segment started or stopped

NOTE: this was designed and tested with a particular version of SMACH, using the introspection server.
'''

name_counts = {}

# 
def parse_labels(msg) :
    label = SegmentLabel()
    label.segment_name = msg.active_states[0]

    if not label.segment_name in name_counts:
        name_counts[label.segment_name] = 1
    elif type(msg) == str and msg.info == 'HEARTBEAT':
        name_counts[label.segment_name] = name_counts[label.segment_name] + 1

    label.segment_id = name_counts[label.segment_name]

    pub.publish(label)


if __name__ == '__main__':

    rospy.init_node('get_smach_info_node')
    topic = rospy.get_param('~topic', '/peg_task_introspection_server/smach/container_status')
    publish_topic = rospy.get_param('~output_segment', '/segment_labels')

    sub = rospy.Subscriber(topic, SmachContainerStatus, parse_labels)
    pub = rospy.Publisher(publish_topic, SegmentLabel)

    rospy.spin()

#!/usr/bin/env python


import rospy
import rosbag

from smach_msgs.msg import SmachContainerStatus
from collab_msgs.msg import SegmentLabel

'''
read from a rosbag
get labels for different segments
tell us when a segment started or stopped

NOTE: this was designed and tested with a particular version of SMACH, using the introspection server.
'''

# 
def parse_labels(msg) :
    pass


if __name__ == '__main__':

    rospy.init_node('get_smach_info_node')
    topic = rospy.get_param('~topic', '/peg_task_introspection_server/smach/container_status')
    publish_topic=  rospy.get_param('~output_segment', '/segment_labels')

    sub = rospy.Subscriber(topic, SmachContainerStatus, parse_labels)
    pub = rospy.Publisher(publish_topic, SegmentLabel)

    rospy.spin()

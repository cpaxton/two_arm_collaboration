#!/usr/bin/env python

import rospy
import tf
import yaml


if __name__ == "__main__":

    rospy.init_node("inverse_drop_publisher")
    filename = rospy.get_param("~filename")
    spin_rate = rospy.get_param("~rate",10)
    frames_to_capture = rospy.get_param("~frames_to_capture",10)
    wait_time = rospy.get_param("~wait", 1.0)
    ref = rospy.get_param("~reference_frame", "peg2/base_link")

    f = open(filename, 'r')

    data = yaml.load(f)

    broadcaster = tf.TransformBroadcaster()
    rate = rospy.Rate(spin_rate)

    try:

        while not rospy.is_shutdown():

            i = 0
            for (trans, rot) in data:
                i = i + 1
                frame = "drop_point" + str(i)

                broadcaster.sendTransform(trans, rot, rospy.Time.now(), frame, ref)

            rate.sleep()


    except rospy.ROSInterruptException: pass

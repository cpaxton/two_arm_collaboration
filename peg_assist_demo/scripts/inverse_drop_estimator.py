#!/usr/bin/env python

import rospy
import tf

if __name__ == "__main__":

    rospy.init_node("inverse_drop_collector")
    filename = rospy.get_param("~filename")
    capture_rate = rospy.get_param("~rate",10)
    frames_to_capture = rospy.get_param("~frames_to_capture",50)
    wait_time = rospy.get_param("~wait", 5)

    rospy.loginfo("Starting inverse physics estimator for dropping objects")

    rate = rospy.Rate(capture_rate)

    rospy.sleep(wait_time)

    # spin and write out a file
    for i in range(0,frames_to_capture):

        if rospy.is_shutdown():
            break

        rospy.loginfo("Capturing pose at step %d", i)
        rate.sleep()

    


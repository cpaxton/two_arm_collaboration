#!/usr/bin/python

import roslib
import sys
import rospy

from std_srvs.srv import Empty

if __name__ == "__main__":

    rospy.init_node('run_gazebo_script_node')

    print "Started ROS node waiting for Gazebo!"
    sname ="/gazebo/unpause_physics"
    print "Waiting for service: %s"%sname
    rospy.wait_for_service(sname)

    print "Sleeping..."

    # sleep for a bit
    #rospy.sleep(0.25)

    print "Starting physics"
    runscript = rospy.ServiceProxy(sname, Empty)
    runscript()


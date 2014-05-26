#!/usr/bin/python

import roslib
import sys
import rospy

from std_srvs import Empty

if __name__ == "__main__":

    rospy.init_node('run_gazebo_script_node')

    # sleep for a bit
    rospy.sleep(4.)

    sname ="gazebo/unpause_physics"
    print "Waiting for service: %s"%sname
    rospy.wait_for_service(sname)

    runscript = rospy.ServiceProxy(sname, Empty())


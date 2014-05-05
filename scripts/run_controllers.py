#!/usr/bin/python

import roslib
import sys
import rospy

from rtt_ros_msgs.srv import *

def start_controllers (conname, filename):
    sname = conname + "/run_script"
    print "Service name: %s"%sname
    rospy.wait_for_service(sname)
    try:
        runscript = rospy.ServiceProxy(sname, RunScript)
        res = runscript(filename)
        return res.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        sys.exit(0)

def usage():
    return "%s deployer_name path_to_ops_script"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        conname = sys.argv[1]
        filename = sys.argv[2]
        print "Starting %s script %s"%(conname, filename)
        res = start_controllers(conname, filename)
        print "Success: %s"%res

#!/usr/bin/python

import roslib
import sys
import rospy
import tf

from rtt_ros_msgs.srv import *

def start_controllers (conname, filename, world, target):
    sname = conname + "/run_script"
    print "Service name: %s"%sname
    rospy.wait_for_service(sname)
    print "Waiting for transform from %s to %s"%(world, target)

    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():

        rate.sleep()

        try:
            listener = tf.TransformListener()
            listener.waitForTransform(world, target, rospy.Time.now(), rospy.Duration(0.1))
            break
        except tf.Exception, e:
            continue

        

    print "Found TF target frame."

    while not rospy.is_shutdown(): 
        
        rate.sleep()

        try:
            runscript = rospy.ServiceProxy(sname, RunScript)
            res = runscript(filename)
            return res.success
        except rospy.ServiceException, e:
            continue

def usage():
    return "%s deployer_name path_to_ops_script ik_reference_frame ik_target_frame"%sys.argv[0]

if __name__ == "__main__":

    rospy.init_node('run_controllers_script_node')

    if len(sys.argv) >= 5:
        conname = sys.argv[1]
        filename = sys.argv[2]
        world = sys.argv[3]
        target = sys.argv[4]
        print "Starting %s script %s"%(conname, filename)
        res = start_controllers(conname, filename, world, target)
        print "Success: %s"%res
    else:
        print "RUN_CONTROLLERS.PY"
        print "Usage:"
        print usage()
        sys.exit(-1);

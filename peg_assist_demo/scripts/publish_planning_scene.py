#!/usr/bin/env python

import rospy
import std_srvs.srv

'''
PUBLISH_PLANNING_SCENE NODE
This node exists for the sole purpose of calling the ros/gazebo plugin to get the planning scene.
It needs to wait for the Gazebo world to be ready and for all of the objects within it to be loaded.
After that, it simply calls the ros service 
'''

if __name__ == '__main__':
    rospy.init_node('publish_planning_scene_node')
    wait_time = rospy.get_param('~wait_time', 10.0)
    #trigger = rospy.get_param('~wait_for')
    spin_rate = rospy.get_param('~rate', 1)

    try:
        service_name = rospy.get_param('~service')
        service = rospy.ServiceProxy(service_name, std_srvs.srv.Empty)

        service.wait_for_service()

        rospy.loginfo("Starting service for %f seconds", wait_time)

        start = rospy.Time.now()
        end = start + rospy.Duration(wait_time)
        rate = rospy.Rate(spin_rate)

        while rospy.Time.now() < end:

            rospy.loginfo("trying to publish scene information...")

            service()
            rate.sleep()

    except rospy.ROSInterruptException: pass


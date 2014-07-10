#!/usr/bin/env python

import rospy
import std_srvs.srv
import rosnode

'''
PUBLISH_PLANNING_SCENE NODE
This node exists for the sole purpose of calling the ros/gazebo plugin to get the planning scene.
It needs to wait for the Gazebo world to be ready and for all of the objects within it to be loaded.
After that, it simply calls the ros service 
'''

if __name__ == '__main__':
    rospy.init_node('publish_planning_scene_node')
    wait_time = rospy.get_param('~wait_time', 3.0)
    trigger = rospy.get_param('~wait_for')
    spin_rate = rospy.get_param('~rate', 1)
    service_name = rospy.get_param('~service')

    try:
        service = rospy.ServiceProxy(service_name, std_srvs.srv.Empty)

        rospy.loginfo("Waiting for service %s", service_name)
        service.wait_for_service()

        rospy.loginfo("Starting service, will wait for %f seconds", wait_time)
        rate = rospy.Rate(spin_rate)

        while not rospy.is_shutdown(): #rospy.Time.now() < end:

            if trigger in rosnode.get_node_names():

                rospy.loginfo("waiting for %f seconds now that %s is up", wait_time, trigger)
                rospy.sleep(wait_time)
                rospy.loginfo("trying to publish scene information...")
                service()
                break


            rate.sleep()

    except rospy.ROSInterruptException: pass


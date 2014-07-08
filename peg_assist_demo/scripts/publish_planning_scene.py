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
    wait_time = rospy.get_param('wait_time', 10.0)
    service_name = rospy.get_param('service')

    service = rospy.ServiceProxy(service_name, std_srvs.srv.Empty)


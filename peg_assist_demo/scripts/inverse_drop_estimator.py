#!/usr/bin/env python

import rospy
import tf
import yaml

from gazebo_msgs.srv import SetPhysicsProperties
from geometry_msgs.msg import Vector3
from gazebo_msgs.msg import ODEPhysics

if __name__ == "__main__":

    rospy.init_node("inverse_drop_collector")
    filename = rospy.get_param("~filename")
    capture_rate = rospy.get_param("~rate",10)
    frames_to_capture = rospy.get_param("~frames_to_capture",10)
    wait_time = rospy.get_param("~wait", 1.0)
    frame = rospy.get_param("~frame", "ring1/ring_link")
    reference_frame = rospy.get_param("~reference_frame", "peg2/base_link")

    f = open(filename, 'w')

    try:

        rospy.loginfo("Starting inverse physics estimator for dropping objects")
        rate = rospy.Rate(capture_rate)
        tfl = tf.TransformListener()

        rospy.loginfo("Waiting for service to be available...")
        rospy.wait_for_service('/gazebo/set_physics_properties')

        rospy.loginfo("Additional sleep...")
        rospy.sleep(wait_time)

        rospy.loginfo("Setting physics properties...")
        srv = rospy.ServiceProxy("/gazebo/set_physics_properties", SetPhysicsProperties)

        time_step = 0.001
        max_update_rate = 1000.0
        gravity = Vector3()
        gravity.x = 0
        gravity.y = 0
        gravity.z = 9.8
        ode_config = ODEPhysics()
        ode_config.auto_disable_bodies = False
        ode_config.sor_pgs_precon_iters = 0
        ode_config.sor_pgs_w = 1.3
        ode_config.sor_pgs_rms_error_tol = 0.0
        ode_config.contact_surface_layer = 0.001
        ode_config.contact_max_correcting_vel = 100.0
        ode_config.cfm = 0.0
        ode_config.erp = 0.2
        ode_config.max_contacts = 20

        srv(time_step, max_update_rate, gravity, ode_config)

        all_points = []

        # spin and write out a file
        for i in range(0,frames_to_capture):

            if rospy.is_shutdown():
                break

            print "Target frame: " + frame + ", getting transform..."
            tf_done = False

            while not tf_done:
                try:
                    (trans, rot) = tfl.lookupTransform(reference_frame, frame, rospy.Time(0))
                    tf_done = True
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

            print (trans, rot)

            all_points.append((trans, rot))

            rospy.loginfo("Capturing pose at step %d", i)
            rate.sleep()

        print all_points
        f.write(yaml.dump(all_points, default_flow_style=True))

    except rospy.ROSInterruptException: pass

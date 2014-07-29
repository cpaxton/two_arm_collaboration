#!/usr/bin/env python

import rospy
import tf
import yaml

from predicator_msgs.msg import *

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

    pub = rospy.Publisher("/predicator/input", PredicateList)
    vpub = rospy.Publisher("/predicator/valid_input", ValidPredicates)

    msg = PredicateList()
    vmsg = ValidPredicates()

    vmsg.pheader.source = rospy.get_name()
    msg.pheader.source = rospy.get_name()

    vmsg.predicates.append("drop_point")
    vmsg.predicates.append("drop_point_of")

    vmsg.assignments.append(ref)

    i = 0
    for (trans, rot) in data:
        i = i + 1
        frame = "drop_point" + str(i)

        vmsg.assignments.append(frame)

        ps = PredicateStatement()
        ps.predicate = "drop_point"
        ps.num_params = 1
        ps.params[0] = frame
        ps.param_classes.append("location")
        msg.statements.append(ps)

        ps = PredicateStatement()
        ps.predicate = "drop_point_of"
        ps.num_params = 2
        ps.params[0] = frame
        ps.params[1] = ref
        ps.param_classes.append("location")
        ps.param_classes.append("base")
        msg.statements.append(ps)

    pub.publish(msg)
    vpub.publish(vmsg)

    try:

        while not rospy.is_shutdown():

            i = 0
            for (trans, rot) in data:
                i = i + 1
                frame = "drop_point" + str(i)

                broadcaster.sendTransform(trans, rot, rospy.Time.now(), frame, ref)
            pub.publish(msg)
            vpub.publish(vmsg)
            rate.sleep()


    except rospy.ROSInterruptException: pass

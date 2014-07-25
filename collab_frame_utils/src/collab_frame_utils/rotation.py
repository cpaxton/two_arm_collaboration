#!/usr/bin/env python

import rospy
import tf
import tf_conversions as tfc
import PyKDL as pk

verbose = False

def get_frames(num, step, trans, rot, frame=None, off=0):

    f = tfc.fromTf((trans, rot))

    bc = tf.TransformBroadcaster()

    for i in range(1, num+1):
        r = tfc.Rotation(f.M)

        (roll, pitch, yaw) = r.GetRPY()
        #r.DoRotZ(step)
        r = tfc.Rotation.RPY(roll, pitch, yaw + step)

        rf = tfc.Frame(tfc.Rotation.RotZ(step))
        rfz = tfc.Frame(tfc.Rotation.RotY(3.14159))
        #rfz = tfc.Frame(tfc.Rotation.RPY(0,3.14159,0))

        p = rf * tfc.Vector(f.p)
        p2 = rfz * tfc.Vector(p)

        #r2 = tfc.Rotation(f.M)
        r2 = tfc.Rotation.RPY(roll+(3.14159/2), -1*pitch, -1*(yaw + step))
        #r = r.DoRotZ(3.14159)

        f = tfc.Frame(r, p)
        f2 = tfc.Frame(r2, p2)

        (trans2, rot2) = tfc.toTf(f)
        (trans3, rot3) = tfc.toTf(f2)

        if verbose:
            print (trans2, rot2)
            print (trans3, rot3)

        if not frame == None:
            bc.sendTransform(trans2, rot2, rospy.Time.now(), frame2 + "_gen" + str(off+i), frame)
            bc.sendTransform(trans3, rot3, rospy.Time.now(), frame2 + "_flip" + str(off+i), frame)

        if verbose:
            print "---"


if __name__ == "__main__":

    rospy.init_node("ring_rotation_helper_node")

    frame1 = rospy.get_param("~frame1")
    frame2 = rospy.get_param("~frame2")
    num = rospy.get_param("~num", 9)
    step = rospy.get_param("~step", 0.314159*2)
    name = rospy.get_param("~name", "ring1/gen_grasp")

    verbose = rospy.get_param("~verbose", 0) == 1

    start_idx_offset = 1

    listener = tf.TransformListener()

    done_tf = False
    while not done_tf:
        try:
            (trans, rot) = listener.lookupTransform(frame1, frame2, rospy.Time(0))
            done_tf = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    while not rospy.is_shutdown():
        get_frames(num, step, trans, rot, frame1, start_idx_offset)

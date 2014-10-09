#!/usr/bin/env python

import tf
import rospy
import sys

'''
Quick script designed to create CSV files of TF information for however long you want to run it
The point is to let me include this information in MATLAB and play around with it
'''
if __name__ == "__main__":
    rospy.init_node("collab_tf_to_csv")

    if len(sys.argv) < 2:
        sys.exit("ERROR: missing output file name.")
    elif len(sys.argv) < 3:
        sys.exit("ERROR: missing first frame.")
    elif len(sys.argv) < 4:
        sys.exit("ERROR: missing second frame.")

    filename = sys.argv[1]
    frame1 = sys.argv[2]
    frame2 = sys.argv[3]

    rate = rospy.Rate(10.)

    try:
        f = open(filename, "w")
        l = tf.TransformListener()

        while not rospy.is_shutdown():
            try:
                (trans, rot) = l.lookupTransform(frame1,frame2,rospy.Time.now())
                (r, p, y) = tf.transformations.euler_from_quaternion(rot)
                res = ""
                for t in trans:
                    res = res + str(t) + ","
                res = res + str(r) + "," + str(p) + "," + str(y);
                f.write(res)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            rate.sleep()


    except rospy.ROSInterruptException, e:
        pass

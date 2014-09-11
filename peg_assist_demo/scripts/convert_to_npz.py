#!/usr/bin/env python

import rosbag
import numpy as np
import recognizer

if __name__ == '__main__':
    # get command line argument for file

    if len(sys.argv) < 2:
        print "ERROR: not enough arguments!"
        print "Required: [input_file] [output_file]"
    else:
        print sys.argv

        X = []
        Y = []
        labels = {}
        next_label = 0

        rosbag.Bag(sys.argv[0], 'r')

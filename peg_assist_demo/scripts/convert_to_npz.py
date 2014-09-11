#!/usr/bin/env python

import rosbag
import numpy as np
import recognizer
import sys

if __name__ == '__main__':
    # get command line argument for file

    if len(sys.argv) < 3:
        print "ERROR: not enough arguments!"
        print "REQUIRED: python convert_to_npz.py [input_file] [output_file]"
    else:

        X = []
        Y_labels = []
        max_inst = {}
        next_label = 0

        bag = rosbag.Bag(sys.argv[1], 'r')

        for topic, msg, t in bag:
            if topic == '/binary_features':
                name = msg.segment.segment_name
                inst = msg.segment.segment_id
                max_inst[name] = inst
            else:
                print "UNKNOWN TOPIC: " + topic

        for topic, msg, t in bag:
            if topic == '/binary_features':
                name = msg.segment.segment_name
                inst = msg.segment.segment_id
                if not inst == max_inst[name]:
                    name = 'failed_' + name
                
                arr = np.array(msg.value)
                X.append(arr)

                Y_labels.append(name)

        data = {}
        data['X'] = X
        data['labels'] = Y_labels
        filename = sys.argv[2]
        np.savez(filename, data)

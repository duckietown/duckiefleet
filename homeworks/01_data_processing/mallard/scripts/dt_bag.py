#!/usr/bin/env python

import sys
from rosbag import Bag

def dt_bag_decimate():
    input_bag = Bag(sys.argv[1])
    save_freq = int(sys.argv[2])
    output_bag = Bag(sys.argv[3], mode='w')
    
    print("This bag contains " + str(input_bag.get_message_count()) + " messages.")

    i = 0
    for topic, msg, time in input_bag.read_messages():
        if(i % save_freq == 0):
            output_bag.write(topic, msg, time)
        i += 1

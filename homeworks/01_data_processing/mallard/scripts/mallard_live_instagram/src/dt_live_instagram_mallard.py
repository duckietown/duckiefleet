#!/usr/bin/env python

import sys, time
import numpy as np
from scipy.ndimage import filters
import cv2
import roslib
import rospy

from sensor_msgs.msg import CompressedImage

class dt_live_instagram_mallard_node:
    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''

        self.image_pub = rospy.Publisher("/camera/image/compressed/filters/compressed", CompressedImage)

        self.subscriber = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callback,  queue_size = 1)


    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        print 'received image of type: "%s"' % ros_data.format

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

        
        SEPIA_KERNEL = np.matrix('0.272, 0.534, 0.131; 0.349, 0.686, 0.168; 0.393, 0.769, 0.189')
        GRAY_KERNEL = np.matrix('1,0,0;1,0,0;1,0,0')

        filters = rospy.get_param('filter').split(',')

        for filter in filters:
            if(filter == 'grayscale'):
                image_np = cv2.transform(image_np, GRAY_KERNEL)
            elif(filter == 'sepia'):
                image_np = cv2.transform(image_np, SEPIA_KERNEL)
            elif(filter == 'flip_horizontal'):
                image_np = cv2.flip(image_np, 1)
            elif(filter == 'flip_vertical'):
                image_np = cv2.flip(image_np, 0)

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        # Publish new image
        self.image_pub.publish(msg)

def main(args):
    '''Initializes and cleanup ros node'''
    ic = dt-live-instagram-mallard_node()
    rospy.init_node('dt_live_instagram_mallard_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down dt_live_instagram_mallard_node"

if __name__ == '__main__':
    main(sys.argv)

#!/usr/bin/env python

import sys
import cv2
import numpy as np
import os.path

def check_path(path):
    if(not os.path.isfile(path)):
        print("Error! Could not find image at " + path)
        exit(2)

def dt_instagram():
    image_in = cv2.imread(sys.argv[1])
    filters = sys.argv[2].split(':')
    image_out_path = sys.argv[3]
    transformed = image_in.copy()

    SEPIA_KERNEL = np.matrix('0.272, 0.534, 0.131; 0.349, 0.686, 0.168; 0.393, 0.769, 0.189')
    GRAY_KERNEL = np.matrix('1, 0, 0; 1, 0, 0; 1, 0, 0')

    for filter in filters:
        if(filter == 'flip-vertical'):
           transformed = cv2.flip(transformed, 0)
        elif(filter == 'flip-horizontal'):
           transformed = cv2.flip(transformed, 1)
        elif(filter == 'grayscale'):
            transformed = cv2.transform(transformed, GRAY_KERNEL)
        elif(filter == 'sepia'):
            transformed = cv2.transform(transformed, SEPIA_KERNEL)

    print("Writing image to: " + image_out_path)
    cv2.imwrite(image_out_path, transformed)

def dt_image_flip():
    input_path = sys.argv[1]
    output_path = sys.argv[2]

    check_path(input_path)

    if(output_path[-1] != '/'):
        output_path = output_path + '/'

    try:
        os.stat(output_path)
    except:
        os.mkdir(output_path)  

    original_img = cv2.imread(input_path)

    try:
        decoded = cv2.imdecode(original_img, 1)
    except Exception as e:
        print("Error! Could not decode image at " + input_path + ":\n" + str(e))
        exit(3)

    flipped_img = cv2.flip(original_img.copy(), 0)
    side_by_img = np.hstack((original_img, flipped_img))

    def write_files():
        print("Writing images to " + output_path)
        cv2.imwrite(output_path + "regular.jpg", original_img)
        cv2.imwrite(output_path + "flip.jpg", flipped_img)
        cv2.imwrite(output_path + "side_by_side.jpg", side_by_img)

    try:
        write_files()
    except Exception as e:
        print("Unknown error occured:\n" + str(e))
        exit(99)

    #cv2.imshow("flipped", flipped_img)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    #cv2.imshow("side by", side_by_img)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

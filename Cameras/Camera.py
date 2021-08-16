# MIT License
# Copyright (c) 2019 JetsonHacks
# See license
# Using a CSI camera (such as the Raspberry Pi Version 2) connected to a

import time
import cv2

import sys
import numpy

# gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
# Defaults to 1280x720 @ 60fps
# Flip the image by setting the flip_method (most common values: 0 and 2)
# display_width and display_height determine the size of the window on the screen

#Camera array format is BGRx

'''

GOALS:
   Change the print to a ROS Publish
   Use this module as the data in the AI driving (or have 1 camera for driving, and 1 for fusion)

'''




#320
#180

def gstreamer_pipeline(
    capture_width=160,
    capture_height=120,
    display_width=160,
    display_height=120,
    framerate=20,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


def ReturnImageMap():
    # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
    print(gstreamer_pipeline(flip_method=0))
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    if cap.isOpened():
        window_handle = cv2.namedWindow("CSI Camera", cv2.WINDOW_AUTOSIZE)
        print("Cap type: " + str(type(cap)))
        # Window
        ret_val, img = cap.read()
        cv2.imshow("CSI Camera", img)
        print("Image type: " + str(type(img)))
        print("Image array shape: " + str(img.shape))
        print("Image array size: " + str(img.size))
        print("Image array bytes: " + str(img.nbytes))
        print("Image array data type: " + str(img.dtype))
        #print whole array
        numpy.set_printoptions(threshold=sys.maxsize)
        while 1:
            ret_val, img = cap.read()
            #Display  camera
            cv2.imshow("CSI Camera", img)
# !! ! ! ! ! ! ! ! ! ! ! ! ! ! 
            tmp = [img,time.time()]
            #print( img  )  # CHANGE THIS LINE TO A ROS PUBLISH, RATHER THAN PRINT
# ! ! ! ! ! ! ! ! ! ! ! !! ! ! ! 
            # This also acts as
            keyCode = cv2.waitKey(30) & 0xFF
            # Stop the program on the ESC key
            if keyCode == 27:
                break
        cap.release()
        cv2.destroyAllWindows()
    else:
        print("Unable to open camera")
  

if __name__ == "__main__":
    ReturnImageMap()

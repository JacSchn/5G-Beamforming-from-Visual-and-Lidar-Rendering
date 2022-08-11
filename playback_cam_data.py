#!/usr/bin/env python3

#Playback the captured camera data.
#A video will appear showing each image while the timestamp related to the image will appear on console
#Data array can be 1D or 3D.


import numpy as np
import cv2
import os

def gstreamer_pipeline(
    capture_width=640,
    capture_height=360,
    display_width=640,
    display_height=360,
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

def GetImage(i):
    file = np.load('/home/musk/data/front_usb/usb_data_%i.npz' % i)
    #np.set_printoptions(threshold=np.inf)
    print(file['arr_0'])
    return file['arr_1'].reshape(360,640,1)


def ReturnImageMap():
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    if cap.isOpened():
        window_handle = cv2.namedWindow("CSI Camera", cv2.WINDOW_AUTOSIZE)
        for i in range (0, len(os.listdir('/home/musk/data/front_usb/'))):
            cv2.imshow("CSI Camera", GetImage(i))
            keyCode = cv2.waitKey(30) & 0xFF

            if keyCode == 27:
                break
        cap.release()
        cv2.destroyAllWindows()
    else:
        print("Unable to open camera")

def printTimeStampOnly():
    for i in range (0, len(os.listdir('/home/musk/data/front_usb/'))):
        file = np.load('/home/musk/data/front_usb/usb_data_%i.npz' % i)
        print(file['arr_0'])
        print(file['arr_1'])

if __name__=="__main__":
    printTimeStampOnly()

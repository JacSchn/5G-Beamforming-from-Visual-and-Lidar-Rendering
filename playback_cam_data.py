#!/usr/bin/env python3

import numpy as np
import cv2
import os

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

def GetImage(i):
    file = np.load('/home/xiaor/data/camera/cam_data_%i.npz' % i)
    #np.set_printoptions(threshold=np.inf)
    print(file['arr_0'])
    return file['arr_1'].reshape(120,160,3)


def ReturnImageMap():
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    if cap.isOpened():
        window_handle = cv2.namedWindow("CSI Camera", cv2.WINDOW_AUTOSIZE)
        for i in range (0, len(os.listdir('/home/xiaor/data/camera'))):
            cv2.imshow("CSI Camera", GetImage(i))
            keyCode = cv2.waitKey(30) & 0xFF

            if keyCode == 27:
                break
        cap.release()
        cv2.destroyAllWindows()
    else:
        print("Unable to open camera")



if __name__=="__main__":
    ReturnImageMap()


#!/usr/bin/env python3
import os
import rospy
import csv
import time
import numpy as np
from std_msgs.msg import String
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

'''
Subscriber for Sony IMX322 USB camera

'''

import sys
import argparse
import subprocess

import cv2


def parse_args():
    # Parse input arguments
    desc = 'Capture and display live camera video on Jetson Nano'
    parser = argparse.ArgumentParser(description=desc)
    parser.add_argument('--vid', dest='video_dev',
                        help='device # of USB webcam (/dev/video?) [1]',
                        default=1, type=int)

    parser.add_argument('--dest', dest='data_dest',
                        help='destination folder for camera data',
                        default=None, type=string)

    parser.add_argument(

    args = parser.parse_args()
    return args

class FileCount:
    file_cnt = len(os.listdir('/home/musk/data/camera'))

    def get_count(self):
        return self.file_cnt

    def increment(self):
        self.file_cnt += 1

class CameraTimeStamp:
    cam_ts = None

    def time(self):
        return self.cam_ts

    def update(self, new_ts):
        self.cam_ts = new_ts

'''
args[0] = FileCount
args[1] = CameraTimeStamp
'''
def callback(data, args):
    file_path = "/home/musk/data/camera/cam_data_%s" % str(args[0].get_count(args[0]))
    #data.data = data.data.astype(dtype=np.uint8, copy=False)
    np.savez(file_path, args[1].time(args[1]),data.data.astype(dtype=np.uint8, copy=False))
    args[0].increment(args[0])
    print(args[1].time(args[1]))

def time_callback(data, arg):
    arg.update(arg, float(data.data))

def listener():
    args = parse_args()
    rospy.init_node('usb_cam_sub', anonymous=True)
    rospy.Subscriber('cam_data_time', String, time_callback, callback_args=(CameraTimeStamp))
    rospy.Subscriber('cam_data', numpy_msg(Floats), callback, callback_args=(FileCount,CameraTimeStamp))

    rospy.spin()

if __name__ == '__main__':
    listener()

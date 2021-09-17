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

    parser.add_argument('--dest', dest='data_dest', # ex. "--dest front_usb"
                        help='destination folder for camera data',
                        default=None)

    args = parser.parse_args()
    return args

class FileCount:
    file_path = ""
    file_cnt = 0
    init = True

    def init_count(self, data_dest):
        self.file_path = "/home/musk/data/%s" % data_dest
        self.file_cnt = len(os.listdir(self.file_path))
        print("Init: %i" % self.file_cnt)

    def set_init(self):
        self.init = False

    def get_init(self):
        return self.init

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
args[2] = data_dest
'''
def callback(data, args):
    if args[0].get_init(args[0]):
        print("Initialize count")
        args[0].init_count(args[0], args[2])
        args[0].set_init(args[0])

    file_path = "/home/musk/data/%s/usb_data_%s" % (str(args[2]), str(args[0].get_count(args[0])))
    #data.data = data.data.astype(dtype=np.uint8, copy=False)
    np.savez(file_path, args[1].time(args[1]),data.data.astype(dtype=np.uint8, copy=False))
    args[0].increment(args[0])
    print(args[1].time(args[1]))

def time_callback(data, arg):
    arg.update(arg, float(data.data))

def listener():
# Parse the arguments
    args = parse_args()
    port = args.video_dev
    data_dest = args.data_dest

    port_name = ("usb_port_%s" % str(port))
    time_name = ("ts_port_%s" % str(port))

    rospy.init_node('usb_cam_sub', anonymous=True)
    rospy.Subscriber(time_name, String, time_callback, callback_args=(CameraTimeStamp))
    rospy.Subscriber(port_name, numpy_msg(Floats), callback, callback_args=(FileCount,CameraTimeStamp, data_dest))

    rospy.spin()

if __name__ == '__main__':
    listener()

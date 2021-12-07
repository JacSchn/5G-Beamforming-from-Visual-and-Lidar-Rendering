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
Can be controlled via a web application through the use of app_companion.py
'''

import sys
import argparse
import subprocess

import cv2


class AppCallback:
    def __init__(self, port_name, time_name, data_dest) -> None:
        self.status = "0"
        self.statusSub = rospy.Subscriber('sensor_status', String, statusCallback, callback_args=(port_name, time_name, data_dest))
        self.timeSub = rospy.Subscriber(time_name, String, time_callback, callback_args=(CameraTimeStamp))
        self.camSub = rospy.Subscriber(port_name, numpy_msg(Floats), callback, callback_args=(FileCount,CameraTimeStamp, data_dest, usb))
        
    def statusCallback(self, status):
        self.status = status.data
        print(f'Current status is {self.status}')


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
            return int(self.cam_ts)

        def update(self, new_ts):
            self.cam_ts = new_ts

    class Sensor:
        def __init__(self, stat):
            self.status = stat

    '''
    args[0] = FileCount
    args[1] = CameraTimeStamp
    args[2] = data_dest
    '''
    def callback(self, data, args):
        if self.status == "0":
            return
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


def parse_args():
    # Parse input arguments
    desc = 'Subscriber for the Sony IMX322 USB camera publisher.\nSaves image arrays to a specified folder as a flattened array.'
    parser = argparse.ArgumentParser(description=desc)
    parser.add_argument('--vid', dest='video_dev',
                        help='device # of USB webcam (/dev/video?) [1]',
                        default=1, type=int)

    parser.add_argument('--dest', dest='data_dest', # ex. "--dest front_usb"
                        help='destination folder for camera data',
                        default=None)

    args = parser.parse_args()
    return args


def main():
    args = parse_args()
    port = args.video_dev
    data_dest = args.data_dest
    port_name = ("usb_port_%s" % str(port))
    time_name = ("ts_port_%s" % str(port))

    rospy.init_node('usb_cam_sub', anonymous=True)
    usbData = AppCallback(port_name=port_name, time_name=time_name, data_dest=data_dest)
    rospy.spin()


if __name__ == '__main__':
    main()

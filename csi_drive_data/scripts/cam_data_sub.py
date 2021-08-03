#!/usr/bin/env python3
import os
import rospy
import csv
import time
import numpy as np
from std_msgs.msg import String
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

class FileCount:
    file_cnt = len(os.listdir('/home/xiaor/data/camera'))

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
    file_path = "/home/xiaor/data/camera/cam_data_%s" % str(args[0].get_count(args[0]))
    #data.data = data.data.astype(dtype=np.uint8, copy=False)
    np.savez(file_path, args[1].time(args[1]),data.data.astype(dtype=np.uint8, copy=False))
    args[0].increment(args[0])
    print(args[1].time(args[1]))

def time_callback(data, arg):
    arg.update(arg, float(data.data))

def listener():
    rospy.init_node('cam_data_sub', anonymous=True)
    rospy.Subscriber('cam_data_time', String, time_callback, callback_args=(CameraTimeStamp))
    rospy.Subscriber('cam_data', numpy_msg(Floats), callback, callback_args=(FileCount,CameraTimeStamp))

    rospy.spin()

if __name__ == '__main__':
    listener()

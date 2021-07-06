#!/usr/bin/env python3
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from visual_self_driving_lidar.msg import Cam_msg

# Change this to send data to be processed elsewhere
def callback(data):
    #data = data.reshape((120, 160, 3), order='F')
    rospy.loginfo(rospy.get_caller_id() + "Received camera data %s", str(data.data))

def listener():
    rospy.init_node('cam_data_sub', anonymous=True)

    rospy.Subscriber('cam_data', numpy_msg(Floats), callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

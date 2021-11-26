
#!/usr/bin/env python3
'''
Publisher for Sony IMX322 cameras
'''

import sys
import argparse
import subprocess
import time
import numpy as np
import cv2
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

'''
TODO
1. Strip out non-necessary functions and arguments
2. Add publisher for timestamp and camera data
	The publisher topic must relate to the camera port number
'''

def parse_args():
    # Parse input arguments
    desc = 'Capture and display live camera video on Jetson Nano'
    parser = argparse.ArgumentParser(description=desc)
    parser.add_argument('--usb', dest='use_usb',
                        help='use USB webcam (remember to also set --vid)',
                        action='store_true')
    # Argument for using Sony IMX322 USB camera
    parser.add_argument('--sony', dest='use_sony',
                        help='use Sony IMX322 camera (remember to also set --vid)',
                        action='store_true')
    parser.add_argument('--vid', dest='video_dev',
                        help='device # of USB webcam (/dev/video?) [1]',
                        default=1, type=int)
    parser.add_argument('--width', dest='image_width',
                        help='image width [1280]',
                        default=640, type=int)
    parser.add_argument('--height', dest='image_height',
                        help='image height [720]',
                        default=360, type=int)
    args = parser.parse_args()
    return args

# Open the stream for sony IMX322 USB camera
def open_cam_sony(dev, width, height):
    gst_str = ('v4l2src device=/dev/video{} ! jpegdec ! videoconvert ! appsink').format(dev) # This command works
#    gst_str = "v4l2src 'device=/dev/video1 io-mode=2' ! 'image/jpeg,width=1280,height=720' ! nvjpegdec ! video/x-raw ! nvvidconv ! 'video/x-raw(memory:NVMM),format=I420' ! nvoverlaysink"
    cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
    #cap.set(cv2.CAP_PROP_FPS, 15.000)
    return cap

def open_cam_usb(dev, width, height):
    gst_str = ('v4l2src device=/dev/video{} ! '
               'video/x-raw, width=(int){}, height=(int){} ! '
               'videoconvert ! appsink').format(dev, width, height)
    return cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)

def read_cam(cap, port_name, time_name):
    frame_rate = 60
    prev = 0
    disp_info = True


# Initialize ROS topics to publish to below #
   

    print('Setting up ROS camera publishers...')
    pub = rospy.Publisher(port_name, numpy_msg(Floats), queue_size=4)
    pub_timestamp = rospy.Publisher(time_name, String, queue_size=4)

    rospy.init_node('USB_cam_data_pub', anonymous=True)
    timestamp = None
    print('ROS camera publisher initialized. Topic name is %s' % port_name)
    print('ROS camera timestamp publisher initialized. Topic name is %s' % time_name)
    time.sleep(2)

    while True:

        if rospy.is_shutdown():
            print("Terminating ROS system")
            break

        time_elasped = time.time() - prev

        timestamp = int(time.time()*1000) #timestamp collection. *1000 for ms format

        _, img = cap.read() # grab the next image frame from camera

#        if time_elasped > 1./frame_rate: # This seems to be making things slow. ROS slows things down a lot by itself
            # manual framerate. Only allow rest of code to run if time elapsed is correct for framerate
#            prev = time.time()
#        else:
#            continue

# ROS publisher for camera data and timestamp below #
       
        if not rospy.is_shutdown():
            try:
                pub_timestamp.publish(str(timestamp)) #publish timestamp of cam data
                rospy.loginfo(timestamp)
                img_flat = img.flatten()
                img_flat = img_flat.astype(dtype=np.float32, casting='safe', copy=False)
                # B/W conversion attempt
                if "rgb" in img_flat.encoding:
                    gray_img = cv2.cvtColor(img_flat, cv2.COLOR_RGB2GRAY)
                elif "bgr" in img_flat.encoding:
                    gray_img = cv2.cvtColor(img_flat, cv2.COLOR_BGR2GRAY)
                # Transform back to Image message
                gray_img_msg = self.cv_bridge.cv2_to_imgmsg(
                gray_img, encoding="mono8")
                # B/W converstion attempt end
                pub.publish(gray_img) #publish cam data
            except rospy.ROSInterruptException:
                rospy.logerr("ROS Interrupt Exception! Just ignore the exception!")
            except rospy.ROSTimeMovedBackwardsException:
                rospy.logerr("ROS Time Backwards! Just ignore the exception!")

#       cv2.imshow('Hello There', img) # shows camera image
        key = cv2.waitKey(10)

        # Display camera stream info
        if disp_info:
            print("Image type: " + str(type(img)))
            print("Image array shape: " + str(img.shape))
            print("Image array size: " + str(img.size))
            print("Image array bytes: " + str(img.nbytes))
            print("Image array data type: " + str(img.dtype))
            print("FPS: " + str(cap.get(cv2.CAP_PROP_FPS)))
            print("cv2 Width: " + str(cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
            print("cv2 Height: " + str(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
            print("cv2 Format: " + str(cap.get(cv2.CAP_PROP_MODE)))
            print(str(img))
            disp_info = False

def main():
    args = parse_args()
    print('Called with args:')
    print(args)
    print('OpenCV version: {}'.format(cv2.__version__))

    if args.use_usb:
        cap = open_cam_usb(args.video_dev,
                           args.image_width,
                           args.image_height)
    else: # if Sony IMX322
        cap = open_cam_sony(args.video_dev,
                            args.image_width,
                            args.image_height)

    if not cap.isOpened():
        sys.exit('Failed to open camera!')

    port_name = ("usb_port_%s" % str(args.video_dev))
    time_name = ("ts_port_%s" % str(args.video_dev))
    read_cam(cap, port_name, time_name)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

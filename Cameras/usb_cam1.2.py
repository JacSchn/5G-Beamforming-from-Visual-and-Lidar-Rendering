import sys
import argparse
import subprocess

import os
import time
import numpy as np
from PIL import Image
import glob
import cv2
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String


WINDOW_NAME = 'CameraDemo'


def parse_args():
    # Parse input arguments
    desc = 'Capture and display live camera video on Jetson TX2/TX1'
    parser = argparse.ArgumentParser(description=desc)
    parser.add_argument('--usb', dest='use_usb',
                        help='use USB webcam (remember to also set --vid)',
                        action='store_true')
    parser.add_argument('--vid', dest='video_dev',
                        help='device # of USB webcam (/dev/video?) [1]',
                        default=1, type=int)
    parser.add_argument('--width', dest='image_width',
                        help='image width [1280]',
                        default=1920, type=int)
    parser.add_argument('--height', dest='image_height',
                        help='image height [720]',
                        default=1080, type=int)
    args = parser.parse_args()
    return args

class USBCam:
    def open_cam_usb(dev, width, height):
        gst_str = ('v4l2src device=/dev/video{} ! '
                   'video/x-raw, width=(int){}, height=(int){} ! '
                'videoconvert ! appsink').format(dev, width, height)
        return cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)


    def open_window(width, height):
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, width, height)
        cv2.moveWindow(WINDOW_NAME, 0, 0)
        cv2.setWindowTitle(WINDOW_NAME, 'Camera Demo for Jetson TX2/TX1')
   

    def read_cam(cap):
        show_help = True
        full_scrn = False
        help_text = '"Esc" to Quit, "H" for Help, "F" to Toggle Fullscreen'
        font = cv2.FONT_HERSHEY_PLAIN
        while True:
            if cv2.getWindowProperty(WINDOW_NAME, 0) < 0:
                # Check to see if the user has closed the window
                # If yes, terminate the program
                break
            _, img = cap.read() # grab the next image frame from camera
            if show_help:
                cv2.putText(img, help_text, (11, 20), font,
                            1.0, (32, 32, 32), 4, cv2.LINE_AA)
                cv2.putText(img, help_text, (10, 20), font,
                            1.0, (240, 240, 240), 1, cv2.LINE_AA)
            cv2.imshow(WINDOW_NAME, img)
            key = cv2.waitKey(10)
            if key == 27: # ESC key: quit program
                break
            elif key == ord('H') or key == ord('h'): # toggle help message
                show_help = not show_help
            elif key == ord('F') or key == ord('f'): # toggle fullscreen
                full_scrn = not full_scrn
                if full_scrn:
                    cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN,
                                          cv2.WINDOW_FULLSCREEN)
                else:
                    cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN,
                                          cv2.WINDOW_NORMAL)
                
                
    def pub_setup(self, resolution=(1920, 1080), framerate=60):
        print('Setting up ROS camera publishers...')
        self.pub = rospy.Publisher('USBcam_data', numpy_msg(Floats), queue_size=4)
        self.pub_time = rospy.Publisher('USBcam_data_time', String, queue_size=4)

        rospy.init_node('USBcam_data_pub', anonymous=True)
        self.time = None
        print('ROS camera publisher initialized. Topic name is USBcam_data')
        print('ROS camera timestamp publisher initialized. Topic name is USBcam_data_time')
        time.sleep(2)
        
    def publish(self):
        self.time = time.time()
        if not rospy.is_shutdown():
            try:
                self.pub_time.publish(str(self.time)) #publish timestamp of cam data
                rospy.loginfo(self.time)
                came_flat = came.flatten()
                came_flat = came_flat.astype(dtype=np.float32, casting='safe', copy=False)
                self.pub.publish(came_flat) #publish cam data
            except rospy.ROSInterruptException:
                rospy.logerr("ROS Interrupt Exception! Just ignore the exception!")
            except rospy.ROSTimeMovedBackwardsException:
                rospy.logerr("ROS Time Backwards! Just ignore the exception!")
         
    
def main():
    args = parse_args()
    print('Called with args:')
    print(args)
    print('OpenCV version: {}'.format(cv2.__version__))

    if args.use_usb:
        cap = open_cam_usb(args.video_dev,
                           args.image_width,
                           args.image_height)

    if not cap.isOpened():
        sys.exit('Failed to open camera!')

    open_window(args.image_width, args.image_height)
    read_cam(cap)
    pub_setup(self, resolution=(1920, 1080), framerate=60)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

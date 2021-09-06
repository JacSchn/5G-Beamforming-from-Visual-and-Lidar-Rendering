'''
Script allows use of webcam, sony IMX322 USB camera, and CSI camera



'''
#test
import sys
import argparse
import subprocess
import time

import cv2


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
                        default=1280, type=int)
    parser.add_argument('--height', dest='image_height',
                        help='image height [720]',
                        default=720, type=int)
    args = parser.parse_args()
    return args

# Open the stream for sony IMX322 USB camera
def open_cam_sony(dev, width, height):
#    gst_str = ('v4l2src device=/dev/video{} ! jpegdec ! videoconvert ! appsink').format(dev)
    gst_str = "v4l2src 'device=/dev/video1 io-mode=2' ! 'image/jpeg,width=1280,height=720' ! nvjpegdec ! video/x-raw ! nvvidconv ! 'video/x-raw(memory:NVMM),format=I420' ! nvoverlaysink"
    cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
    #cap.set(cv2.CAP_PROP_FPS, 15.000)
    return cap

def open_cam_usb(dev, width, height):
    # We want to set width and height here, otherwise we could just do:
    #     return cv2.VideoCapture(dev)
    gst_str = ('v4l2src device=/dev/video{} ! '
               'video/x-raw, width=(int){}, height=(int){} ! '
               'videoconvert ! appsink').format(dev, width, height)
    return cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)

def read_cam(cap):
    frame_rate = 15
    prev = 0
    disp_info = True

    while True:
        time_elasped = time.time() - prev
        _, img = cap.read() # grab the next image frame from camera

        if time_elasped > 1./frame_rate:
            # manual framerate. Only allow rest of code to run if time elapsed is correct for framerate
            prev = time.time()
        else:
            continue
        #cv2.imshow(WINDOW_NAME, img)
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

        if key == 27: # ESC key: quit program
            break

def main():
    args = parse_args()
    print('Called with args:')
    print(args)
    print('OpenCV version: {}'.format(cv2.__version__))

    if args.use_usb:
        cap = open_cam_usb(args.video_dev,
                           args.image_width,
                           args.image_height)
    elif args.use_sony: # if Sony IMX322
        cap = open_cam_sony(args.video_dev,
                            args.image_width,
                            args.image_height)

    else: # by default, use the Jetson onboard camera
        cap = open_cam_onboard(args.image_width,
                               args.image_height)

    if not cap.isOpened():
        sys.exit('Failed to open camera!')

    read_cam(cap)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

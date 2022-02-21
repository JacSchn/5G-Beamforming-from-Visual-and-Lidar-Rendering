#!/usr/bin/env python3

#Playback the captured camera data.
#A video will appear showing each image while the timestamp related to the image will appear on console
#Data array can be 1D or 3D.


# import cv2
from email.utils import parsedate
import os
import argparse
from tkinter import N
# from matplotlib.pyplot import title
import numpy as np

# def gstreamer_pipeline(
#     capture_width=640,
#     capture_height=360,
#     display_width=640,
#     display_height=360,
#     framerate=20,
#     flip_method=0,
# ):
#     return (
#         "nvarguscamerasrc ! "
#         "video/x-raw(memory:NVMM), "
#         "width=(int)%d, height=(int)%d, "
#         "format=(string)NV12, framerate=(fraction)%d/1 ! "
#         "nvvidconv flip-method=%d ! "
#         "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
#         "videoconvert ! "
#         "video/x-raw, format=(string)BGR ! appsink"
#         % (
#             capture_width,
#             capture_height,
#             framerate,
#             flip_method,
#             display_width,
#             display_height,
#         )
#     )

# def GetImage(i):
#     file = np.load('/home/musk/data/front_usb/usb_data_%i.npz' % i)
#     #np.set_printoptions(threshold=np.inf)
#     print(file['arr_0'])
#     return file['arr_1'].reshape(360,640,1)


# def ReturnImageMap():
#     cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
#     if cap.isOpened():
#         window_handle = cv2.namedWindow("CSI Camera", cv2.WINDOW_AUTOSIZE)
#         for i in range (0, len(os.listdir('/home/musk/data/front_usb/'))):
#             cv2.imshow("CSI Camera", GetImage(i))
#             keyCode = cv2.waitKey(30) & 0xFF

#             if keyCode == 27:
#                 break
#         cap.release()
#         cv2.destroyAllWindows()
#     else:
#         print("Unable to open camera")

# def printTimeStampOnly():
#     for i in range (0, len(os.listdir('/home/musk/data/front_usb/'))):
#         file = np.load('/home/musk/data/front_usb/usb_data_%i.npz' % i)
#         print(file['arr_0'])
#         print(file['arr_1'])

def parseDual(input: str= None):
    parser = argparse.ArgumentParser(description=argparse.SUPPRESS,usage=argparse.SUPPRESS,epilog=argparse.SUPPRESS,add_help=False, prefix_chars='[')

    parser.add_argument('[cake',help='is cake')
    print(f'\n\n{input}\n\n')
    parser.print_help()

    return parser

def parseArgs():
    '''
    Parse command line arguments.
    Documentation for argparse can be found here https://docs.python.org/3/library/argparse.html.
    '''
    space = '    ' # Hacky method to align the help description for arguments that use '\b' for the metavar value.

    desc = 'View previously captured images that is currently saved in a .npz file on screen.\n  By default the images run continuously.'
    parser = argparse.ArgumentParser(description=desc,formatter_class=argparse.RawTextHelpFormatter)

    parser.add_argument('data_dir', metavar='dataDir', help='filepath of the directory the data is stored in', type=str)


    parser.add_argument('-W','--width', dest='width',
                        help=f'{space}capture width of image (default: %(default)s)',
                        default=640, type=int, metavar='\b') # The value of metavar changes the default help display from "-W WIDTH, --width WIDTH" to "-W, --width"
                                                             # \b is a backspace character. An empty string results in an extra space being added.

    parser.add_argument('-H','--height', dest='height',
                        help=f'{space}capture height of image (default: %(default)s)',
                        default=360, type=int, metavar='\b')
    
    parser.add_argument('-v','--version', action='version',version='%(prog)s\nVersion 1.0')

    '''
    Optional flags
    Takes no value
    '''
    opt_flags = parser.add_argument_group(title='optional flags')

    opt_flags.add_argument('--color', dest='is_color',
                        help='enables color mode\n  Note: Only works with data that was originally captured in color',
                        action='store_true')

    opt_flags.add_argument('-s','--step', dest='step',
                        help=f'{space}step size', metavar='\b')

    '''
    Options for when specifying a second display.
    Useful for when wanting to show multiple camera images to the screen at once.
    '''

    

    secondDisp = parser.add_argument_group(title='Second Display',description='Additional flags to use when -d or --dual was specified')


    
    
    secondDisp.add_argument('-d','--dual', help=f'{space}display a second set of images', action=CustomAction, additional_arg1='[foo', additional_arg2='--bar', nargs='?', default=argparse.SUPPRESS)

    

    args = parser.parse_args()
    
    print(args)
    # if args.video_dev == None:
    #     parser.error('--vid Must be set!')
    # if args.data_dest == None:
    #     parser.error('--dest Must be set!')
    # if args.name == None:
    #     parser.error('--name Must be set!')

    return args


def main():
    args = parseArgs()
    # port_name = ("usb_port_%s" % args.video_dev)
    # time_name = ("ts_port_%s" % str(args.video_dev))
    # node_name = ('%s_port_%s' % (args.name, args.video_dev))

if __name__ == '__main__':
    main()


#TODO Add dual run mode. See both front and rear images at the same time
#TODO Add flag and function to convert numpy array to an image and save it in a user specified directory.





'''
Junk Code

    class CustomAction(argparse.Action):
        def __init__(self, option_strings, additional_arg1, additional_arg2,*args, **kwargs):
            self._a1 = additional_arg1
            self._a2 = additional_arg2
            super(CustomAction, self).__init__(option_strings=option_strings,*args, **kwargs)
            
        def __call__(self, parser: argparse.ArgumentParser, namespace, values: str, option_string=None):
            print(self._a1)
            print(self._a2)
            print(option_string)
            print(str(values))
            print(namespace)
            parser.prefix_chars = '['
            secondDisp.add_argument(self._a1, help='has foo', default='Major Foo')
            #parser.parse_args(str(values))
            setattr(namespace, self.dest, values)

'''
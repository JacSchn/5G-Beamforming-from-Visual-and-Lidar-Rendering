#!/usr/bin/env python3

#Playback the captured camera data.
#A video will appear showing each image while the timestamp related to the image will appear on console
#Data array can be 1D or 3D.


# import cv2
from msilib.schema import Class
import os
import argparse
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

def parentParser(space: str, pfx: str = '') -> argparse.ArgumentParser:
    parent = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter, add_help=False)

    parent.add_argument('-sd', '--saveDest', dest=f'{pfx}save_dest', metavar='\b', type=str, nargs=1, 
                            help=f'{space}save directory when the --save flag is set')
    
    parent.add_argument('-fp', '--filePref', dest=f'{pfx}file_prefix', metavar='\b', type=str, nargs=1, 
                            help=f'{space}file prefix that the image will be saved with')

    return parent

def parseDual(parser: argparse.ArgumentParser, space: str) -> None:
    '''
    Options for when specifying a second display.\n
    Useful for when wanting to show multiple camera images to the screen at once.
    '''
    subparser = parser.add_subparsers(dest="sec_disp_cmd", title="Second Display", metavar='DUAL',
                                    description="run '%(prog)s . dual -h' for a full list of arguments")

    sec_disp_sub = subparser.add_parser('dual', formatter_class=argparse.RawTextHelpFormatter, parents=[parentParser(space=space, pfx='sec_')], 
                                        help='display images from another directory',
                                        usage='%(prog)s dataDir dual [-h] [-sd [-fp]] secDataDir') # Need custom usage to indicate --sd and --fp are mutually inclusive
    
    sec_disp_sub.add_argument('dataDir', metavar='secDataDir', type=str, nargs=1,
                            help='filepath of the directory the data is stored in')

def parseArgs() -> argparse.Namespace:
    '''
    Parse command line arguments.\n
    Documentation for argparse can be found here https://docs.python.org/3/library/argparse.html.
    '''
    space = '    ' # Hacky method to align the help description for arguments that use '\b' for the metavar value.

    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter, parents=[parentParser(space)],
                                    usage='%(prog)s [-h] [-w] [-ht] [-s] [-sv [-sd [-fp]]] [-v] [--color] dataDir DUAL ...', # Need custom usage to indicate -s, -sd, -fp are mutually inclusive
                                    description='View previously captured images that is currently saved in a .npz file on screen.\n  By default the images run continuously.')
    
    parser.add_argument('data_dir', metavar='dataDir', type=str,
                        help='filepath of the directory the data is stored in')
    
     # The value of metavar changes the default help display from "-W WIDTH, --width WIDTH" to "-W, --width"
     # \b is a backspace character. An empty string results in an extra space being added.
    parser.add_argument('-w','--width', dest='width', metavar='\b', default=640, type=int,
                        help=f'{space}capture width of image (default: %(default)s)')
                                                             
    parser.add_argument('-ht','--height', dest='height', metavar='\b', default=360, type=int,
                        help=f'{space}capture height of image (default: %(default)s)')
    
    parser.add_argument('-s','--step', dest='step', metavar='\b', default=10, type=int, choices=range(1,21),
                        help=f'{space}step through images\n  step size (default: %(default)s)\n  possible choices [1..20]')

    parser.add_argument('-v','--version', action='version', version='%(prog)s\nVersion 1.0')

    '''
    Optional flags
    Takes no value
    '''
    opt_flags = parser.add_argument_group(title='optional flags')

    opt_flags.add_argument('--color', dest='is_color', action='store_true',
                        help='enables color mode (default: %(default)s)\n  Note: Only works with data that was originally captured in color')

    opt_flags.add_argument('-sv', '--save', dest='will_save', action='store_true',
                        help='save images (default: %(default)s)')

    parseDual(parser=parser, space=space) # Subparser for second display arguments

    args = parser.parse_args() # Extract argument values

    # Check validity of arguments that are mutually inclusive
    if args.will_save:
        if not args.save_dest and not args.file_prefix:
            parser.error('the following arguments are required: -sd -fp')
        if args.save_dest and not args.file_prefix:
            parser.error('the following arguments are required: -fp')
        if  args.file_prefix and not args.save_dest:
            parser.error('the following arguments are required: -sd')

    if args.will_save and args.sec_disp_cmd:
        if not args.sec_save_dest and not args.sec_file_prefix:
            parser.error('the following arguments are required for dual: -sd -fp')
        if args.sec_save_dest and not args.sec_file_prefix:
            parser.error('the following arguments are required for dual: -fp')
        if  args.sec_file_prefix and not args.sec_save_dest:
            parser.error('the following arguments are required for dual: -sd')

    return args

class ImageData:
    


def main():
    args = parseArgs()
    # port_name = ("usb_port_%s" % args.video_dev)
    # time_name = ("ts_port_%s" % str(args.video_dev))
    # node_name = ('%s_port_%s' % (args.name, args.video_dev))

if __name__ == '__main__':
    main()


#TODO Add dual run mode. See both front and rear images at the same time
#TODO Add flag and function to convert numpy array to an image and save it in a user specified directory.
#TODO Add sync flag to automatically sync up the two outputs when displaying two at the same time. Can be done by looking at the timestamp
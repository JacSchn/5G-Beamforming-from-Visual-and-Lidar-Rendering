#!/usr/bin/env python3

#Playback the captured camera data.
#A video will appear showing each image while the timestamp related to the image will appear on console
#Data array can be 1D or 3D.


from dis import dis
import cv2
import os
import argparse
import numpy as np

def parent_parser(space: str, pfx: str = '') -> argparse.ArgumentParser:
    parent = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter, add_help=False)
        
    parent.add_argument(f'{pfx}data_src', metavar='dataDir', type=str, nargs=1,
                        help='filepath of the directory the data is stored in')

    parent.add_argument('-sd', '--saveDest', dest=f'{pfx}save_dest', metavar='\b', type=str, nargs=1, 
                            help=f'{space}save directory when the --save flag is set')
    
    parent.add_argument('-fp', '--filePref', dest=f'{pfx}file_prefix', metavar='\b', type=str, nargs=1, 
                            help=f'{space}file prefix that the image will be saved with\n  Ex: cam_img_')

    return parent

def parent_dual(parser: argparse.ArgumentParser, space: str) -> None:
    '''
    Options for when specifying a second display.\n
    Useful for when wanting to show multiple camera images to the screen at once.
    '''
    subparser = parser.add_subparsers(dest="sec_disp_cmd", title="Second Display", metavar='DUAL',
                                    description="run '%(prog)s . dual -h' for a full list of arguments")

    subparser.add_parser('dual', formatter_class=argparse.RawTextHelpFormatter, parents=[parent_parser(space=space, pfx='sec_')], 
                                        help='display images from another directory',
                                        usage='%(prog)s dataDir dual [-h] [-sd [-fp]] dataDir') # Need custom usage to indicate --sd and --fp are mutually inclusive

def parse_args() -> argparse.Namespace:
    '''
    Parse command line arguments.\n
    Documentation for argparse can be found here https://docs.python.org/3/library/argparse.html.
    '''
    space = '    ' # Hacky method to align the help description for arguments that use '\b' for the metavar value.

    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter, parents=[parent_parser(space)],
                                    usage='%(prog)s [-h] [-w] [-ht] [-s] [-sv [-sd [-fp]]] [-v] [--color] dataDir DUAL ...', # Need custom usage to indicate -s, -sd, -fp are mutually inclusive
                                    description='View previously captured images that is currently saved in a .npz file on screen.\n  By default the images run continuously.')
    
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

    parent_dual(parser=parser, space=space) # Subparser for second display arguments

    args = parser.parse_args() # Extract argument values
    print(f"\n{args}\n")
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
    def __init__(self, data_src: str, sv_dest: str=None, file_pfx: str=None):
        self.src = data_src[0]
        self.dest = sv_dest
        self.file_pfx = file_pfx

class Display:
    def __init__(self, w: int, h: int, step: int, color: bool, save: bool):
        self.width = w
        self.height = h
        self.depth = 3 if color else 1 # Depth of array. 3 for color. 1 for B/W
        self.step = step
        self.is_color = color
        self.will_save = save

def init_image_data() -> tuple[ImageData, Display]:
    args = parse_args()
    
    cam1 = None
    cam2 = None

    if args.will_save:
        cam1 = ImageData(args.data_src, args.save_dest, args.file_prefix)
        if args.sec_disp_cmd: # True if sec image dir was specified
            cam2 = ImageData(args.sec_data_src, args.sec_save_dest, args.sec_file_prefix)
    else:
        cam1 = ImageData(args.data_src)
        if args.sec_disp_cmd:
            cam2 = ImageData(args.sec_data_src)

    ret_cam = [cam1, cam2] if args.sec_disp_cmd else [cam1]

    return ret_cam, Display(args.width, args.height, args.step, args.is_color, args.will_save)
    
def GetImage(i):
    file = np.load(r'C:\Users\sflyn\Documents\Research Project\Jetson Car\Data\Data-10-29-21\front_usb\usb_data_%i.npz' % i)
    #np.set_printoptions(threshold=np.inf)
    print(file['arr_0'])
    return file['arr_1'].reshape(360,640,3)

def GetRearImage(i):
    file = np.load(r'C:\Users\sflyn\Documents\Research Project\Jetson Car\Data\Data-10-29-21\rear_usb\usb_data_%i.npz' % i)
    #np.set_printoptions(threshold=np.inf)
    print(file['arr_0'])
    return file['arr_1'].reshape(360,640,3)

def get_nparray(i: int, id: ImageData, d: Display) -> np.ndarray:
    file = np.load(rf'{id.src}\usb_data_{i}.npz')
    return file['arr_1'].reshape(d.height, d.width, d.depth)

def display(imd: list, d: Display) -> None:
    
    cv2.namedWindow("Generic Cam 0", cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow("Generic Cam 1", cv2.WINDOW_AUTOSIZE)
    print(imd)
    for i in range(0, len(os.listdir(r'C:\Users\sflyn\Documents\Research Project\Jetson Car\Data\Data-10-29-21\front_usb'))):
        for j, im_data in enumerate(imd):
            cv2.imshow(f"Generic Cam {j}", get_nparray(i, im_data, d))
        
        keyCode = cv2.waitKey(30) & 0xFF
        
        if keyCode == 27:
            break

    cv2.destroyAllWindows()
    

    # if cap.isOpened():
    #     window_handle = cv2.namedWindow("CSI Camera", cv2.WINDOW_AUTOSIZE)
    #     for i in range (0, len(os.listdir('front_usb'))):
    #         cv2.imshow("CSI Camera", GetImage(i))
    #         keyCode = cv2.waitKey(30) & 0xFF

    #         if keyCode == 27:
    #             break
    #     cap.release()
    #     cv2.destroyAllWindows()
    # else:
    #     print("Unable to open camera")

def printTimeStampOnly():
    for i in range (0, len(os.listdir('/home/musk/data/front_usb/'))):
        file = np.load('/home/musk/data/front_usb/usb_data_%i.npz' % i)
        print(file['arr_0'])
        print(file['arr_1'])

def main():
    image, disp = init_image_data()
    display(image, disp)


if __name__ == '__main__':
    main()


#TODO Add dual run mode. See both front and rear images at the same time
#TODO Add flag and function to convert numpy array to an image and save it in a user specified directory.
#TODO Add sync flag to automatically sync up the two outputs when displaying two at the same time. Can be done by looking at the timestamp
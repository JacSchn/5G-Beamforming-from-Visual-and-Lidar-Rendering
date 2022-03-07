#!/usr/bin/env python3

# View previously saved image array data.
#Data array can be 1D or 3D.

import cv2
import os
import argparse
import numpy as np

def parse_args() -> argparse.Namespace:
    '''
    Parse command line arguments.\n
    Documentation for argparse can be found here https://docs.python.org/3/library/argparse.html.
    '''
    space = '    ' # Hacky method to align the help description for arguments that use '\b' for the metavar value.

    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter,
                                   
                                    description='View previously captured images that is currently saved in a .npz file on screen.\n  By default the images run continuously.')
    
     # The value of metavar changes the default help display from "-W WIDTH, --width WIDTH" to "-W, --width"
     # \b is a backspace character. An empty string results in an extra space being added.

    parser.add_argument('data_src', metavar='dataDir', type=str, nargs='+',
                        help='filepath of the directory(s) the data is stored in')

    parser.add_argument('-n','--num-src', dest='num_src', metavar='\b', default=1, type=int, choices=range(1,5),
                        help=f'{space}number of image sources (default: %(default)s)\n  possible choices [1..4]')

    parser.add_argument('-sd', dest='save_dest', metavar='', default=[], type=str, nargs='+', 
                            help='save directory when the --save flag is set')
    
    group = parser.add_mutually_exclusive_group()

    group.add_argument('-fp', dest='file_prefix', metavar='', default=[], type=str, nargs='+', 
                            help='file prefix that the image will be saved with\n  Ex: cam_img_')

    group.add_argument('-FP', dest='uni_file_prefix', metavar='', default='cam_img_', type=str, 
                            help='file prefix that will be used for ALL images being saved (default: %(default)s)')
    
    parser.add_argument('-w','--width', dest='width', metavar='\b', default=640, type=int,
                        help=f'{space}capture width of image (default: %(default)s)')
                                                             
    parser.add_argument('-ht','--height', dest='height', metavar='\b', default=360, type=int,
                        help=f'{space}capture height of image (default: %(default)s)')

    parser.add_argument('-wn', dest='win_name', metavar='', default=[], type=str, nargs='+',
                        help=f'{space}window name for image(s)')
    
    parser.add_argument('-s','--step', dest='step', metavar='\b', default=1, type=int, choices=range(1,51),
                        help=f'{space}step through images\n  step size (default: %(default)s)\n  possible choices [1..50]')

    parser.add_argument('-v','--version', action='version', version='%(prog)s\nVersion 1.0')

    '''
    Optional flags
    Takes no value
    '''
    opt_flags = parser.add_argument_group(title='optional flags')

    opt_flags.add_argument('--color', dest='depth', action='store_const', default=1, const=3,
                        help='enables color mode\n  Note: Only works with data that was originally captured in color')

    opt_flags.add_argument('-sv', '--save', dest='will_save', action='store_true',
                        help='save images (default: %(default)s)')

    args = parser.parse_args() # Extract argument values
    
    print(args)

    if args.num_src != len(args.data_src):
        parser.error('the number of dataDir must equal --num-src')
    if len(args.win_name) > 0 and args.num_src != len(args.win_name):
        parser.error('the number of -wn must equal --num-src')

    if args.will_save:
        if args.num_src != len(args.save_dest):
            parser.error('the number of arguments for -sd must equal -n')
        if len(args.file_prefix) > 0 and args.num_src != len(args.file_prefix):
            parser.error('the number of arguments for -fp must equal -n')

    return args

class ImageData:
    counter = 0
    def __init__(self, data_src: str, sv_dest: str=None, file_pfx: str=None, win_name: str=None):
        self.src = data_src
        self.dest = sv_dest
        self.file_pfx = file_pfx
        ImageData.counter += 1
        self.win_name = win_name if win_name != None else f'Generic Cam {ImageData.counter}'

    def set_win_name(self, wn: str):
        self.win_name = wn

class Display:
    def __init__(self, w: int, h: int, depth: int, step: int, save: bool):
        self.width = w
        self.height = h
        self.depth = depth
        self.step = step
        self.save = save

def init_image_data() -> tuple[ImageData, Display]:
    args = parse_args()
    cams = []

    if args.will_save and len(args.file_prefix) != 0:
        for i in range(0, args.num_src):
            cams.append(ImageData(args.data_src[i], args.save_dest[i], args.file_prefix[i]))
    elif args.will_save: # Use universal file prefix
        for i in range(0, args.num_src):
            cams.append(ImageData(args.data_src[i], args.save_dest[i], args.uni_file_prefix))
    else:
        for i in range(0, args.num_src):
            cams.append(ImageData(args.data_src[i]))

    if len(args.win_name) != 0:
        for i in range(0, args.num_src):
            cams[i].set_win_name(args.win_name[i])

    return cams, Display(args.width, args.height, args.depth, args.step,  args.will_save)
    
def get_nparray(i: int, id: ImageData, d: Display) -> np.ndarray:
    file = np.load(os.path.join(id.src, f'usb_data_{i}.npz'))
    return file['arr_1'].reshape(d.height, d.width, d.depth)

def display(imd: list, disp: Display) -> None:
 
    for im_data in imd:
        cv2.namedWindow(im_data.win_name, cv2.WINDOW_AUTOSIZE)
    
    dir_sizes = []
    for im in imd:
        dir_sizes.append(len(os.listdir(im.src)))

    max_dir_size = max(dir_sizes)
    
    for i in range(0, max_dir_size, disp.step):
        for j, im_data in enumerate(imd):
            if i >= dir_sizes[j]:
                continue
            nparr = get_nparray(i, im_data, disp)
            cv2.imshow(im_data.win_name, nparr)
            if disp.save:
                cv2.imwrite(os.path.join(im_data.dest,f'{im_data.file_pfx}{i}.jpeg'), nparr)
                
        if disp.step > 1:
            input()

        keyCode = cv2.waitKey(20) & 0xFF # FPS of video being shown
    
        if keyCode == 27:
            break
                
    cv2.destroyAllWindows()

def printTimeStampOnly():
    for i in range (0, len(os.listdir('/home/musk/data/front_usb/'))):
        file = np.load('/home/musk/data/front_usb/usb_data_%i.npz' % i)
        print(file['arr_0'])
        print(file['arr_1'])

def main():
    cams, disp = init_image_data()
    display(cams, disp)


if __name__ == '__main__':
    main()

# TODO Add sync flag to automatically sync up the two outputs when displaying two at the same time. Can be done by looking at the timestamp
# TODO Add checks to ensure the directories that are inputted exist.
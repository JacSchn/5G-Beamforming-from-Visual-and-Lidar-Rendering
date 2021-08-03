#!/usr/bin/env python3

#Camera data is saved by default as a 1D array.
#Use if camera data needs to be reshaped into standard form (120,160,3)

import numpy as np
import cv2
import os

for i in range (0, len(os.listdir('/home/musk/data/camera'))):
    file_path = ('/home/musk/data/camera/cam_data_%i.npz' % i)
    file = np.load(file_path)
    file['arr_1'].reshape(120,160,3)
    np.savez(file_path, file['arr_0'], file['arr_1'])
    file2 = np.load(file_path)
    print(file2['arr_1'])



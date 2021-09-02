#!/bin/sh

# Deletes all saved data for all collection methods

# Remove directories with data
rm -r $HOME/data/csi_camera/
rm -r $HOME/data/lidar/
rm -r $HOME/data/front_usb/
rm -r $HOME/data/rear_usb/
#rm -r $HOME/data/

# Create directories
mkdir $HOME/data/csi_camera
mkdir $HOME/data/lidar
mkdir $HOME/data/front_usb/
mkdir $HOME/data/rear_usb/

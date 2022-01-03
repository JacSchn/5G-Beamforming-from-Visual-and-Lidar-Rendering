#!/bin/sh

# Deletes all saved data for all collection methods

# Remove data from files
rm -r $HOME/data/lidar/*
echo "Removed LiDAR data"

cd front_usb/
ls | grep -P "usb_data_[0-9]*.npz" | xargs -d "\n" rm
echo "Removed front USB data"

cd ../rear_usb/
ls | grep -P "usb_data_[0-9]*.npz" | xargs -d "\n" rm
cd ../
echo "Removed rear USB data"

rm -r $HOME/data/server_router/*
echo "Removed server router data"

rm -r $HOME/data/client_router/*
echo "Removed client router data"


# A failed rm command is 1. Good is 0

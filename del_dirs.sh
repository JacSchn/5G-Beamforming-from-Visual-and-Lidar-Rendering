#!/bin/sh

# Deletes all saved data for all collection methods

# Remove data from files
rm -r $HOME/data/lidar/*

cd front_usb/
ls | grep -P "usb_data_[0-9]*.npz" | xargs -d "\n" rm

cd ../rear_usb/
ls | grep -P "usb_data_[0-9]*.npz" | xargs -d "\n" rm
cd ../

rm -r $HOME/data/server_router/*
rm -r $HOME/data/client_router/*

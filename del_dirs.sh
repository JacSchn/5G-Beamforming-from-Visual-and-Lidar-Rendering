#!/bin/sh

# Deletes all saved data for all collection methods

# Remove data from files
rm -r $HOME/data/lidar/*
rm -r $HOME/data/front_usb/*
rm -r $HOME/data/rear_usb/*
rm -r $HOME/data/server_router/*
rm -r $HOME/data/client_router/*

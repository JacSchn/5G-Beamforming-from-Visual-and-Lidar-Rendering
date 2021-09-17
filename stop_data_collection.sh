#!/bin/bash

# Stop all data collection scripts

echo -e "Stopping at time (ms): `date +%s%3N`\n"
echo -e "Stopping at time (ms): `date +%s%3N`\n" >> ~/logs/collection_end

# User has to hit Ctrl+C for several processes to exit cleanly when using gnome-terminal
# echo -e "Hit Ctrl+C in LiDAR, CSI cam, and USB cam terminals"

PID=`cat ~/logs/lidar/pid_data`
sudo kill -2 ${PID}

PID=`cat ~/logs/front_usb/pid_data`
sudo kill -2 ${PID}

PID=`cat ~/logs/rear_usb/pid_data`
sudo kill -2 ${PID}

PID=`cat ~/logs/lidar/pid_launch`
sudo kill -2 ${PID}

PID=`cat ~/logs/front_usb/pid_launch`
sudo kill -2 ${PID}

PID=`cat ~/logs/rear_usb/pid_launch`
sudo kill -2 ${PID}


# Stop server and client router data collection
ssh root@192.168.1.7 '~/stop_collect_data.sh'


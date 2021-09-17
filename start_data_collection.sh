#!/bin/bash

# Start Data Collection

## Can add a
## ; $SHELL
## to the end of the bash command to keep the terminal open when the task terminates
## Ex:
##	"bash -ic 'rosrun rplidar_ros captureRPLiDAR; #SHELL'"

echo -e "Starting Data Collection...\n"
sleep 2

# Start Server and Client router data collection scripts
# gnome-terminal -q --window --title="Server Router" -e "bash -ic 'ssh root@192.168.1.7 eval \"~/collect_data.sh; /bin/sh\"'"

# Start LiDAR data collection
rosrun rplidar_ros captureRPLiDAR > ~/logs/lidar/capture_log 2>&1 &
echo $! > ~/logs/lidar/pid_data
# gnome-terminal -q --window --title="LiDAR Data" -e "bash -ic 'rosrun rplidar_ros captureRPLiDAR'"

# Start CSI Camera data collection
# gnome-terminal -q --tab --title="CSI Camera Data" -e "bash -ic 'rosrun csi_drive_data cam_data_sub.py'"

# Start Front USB Camera data collection
rosrun usb_cam cam_sub.py --vid 0 --dest front_usb > ~/logs/front_usb/usb_collection 2>&1 &
echo $! > ~/logs/front_usb/pid_data
# gnome-terminal -q --tab --title="USB Cam 1" -e "bash -ic 'rosrun usb_cam_data <insert_filename_here>'"

# Start Rear USB Camera data collection
rosrun usb_cam cam_sub.py --vid 2 --dest rear_usb > ~/logs/rear_usb/usb_collection 2>&1 &
echo $! > ~/logs/rear_usb/pid_data
# gnome-terminal -q --tab --title="USB Cam 2" -e "bash -ic 'rosrun usb_cam_data <insert_filename_here>'"

# Time when all collection methods are up
echo -e "Finished at time (ms): `date +%s%3N`\n"
echo -e "Finished at time (ms): `date +%s%3N`\n" >> ~/logs/start_time

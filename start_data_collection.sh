#!/bin/bash

# Start Data Collection

boot_message(){
	echo "Started $1 data collection with PID of $2"
}

echo -e "Starting Data Collection...\n"
sleep 2

# Start Server and Client router data collection scripts
ssh root@192.168.1.7 '~/collect_data.sh'

# Start LiDAR data collection
rosrun rplidar_ros captureRPLiDAR > ~/logs/lidar/capture_log 2>&1 &
echo $! > ~/logs/lidar/pid_data
boot_message "LiDAR" "$!"

# Start Front USB Camera data collection
rosrun usb_cam cam_sub.py --vid 0 --dest front_usb > ~/logs/front_usb/usb_collection 2>&1 &
echo $! > ~/logs/front_usb/pid_data
boot_message "Front USB" "$!"

# Start Rear USB Camera data collection
rosrun usb_cam cam_sub.py --vid 2 --dest rear_usb > ~/logs/rear_usb/usb_collection 2>&1 &
echo $! > ~/logs/rear_usb/pid_data
boot_message "Rear USB" "$!"

# Time when all collection methods are up
echo -e "Started at time (ms): `date +%s%3N`\n"
echo -e "Started at time (ms): `date +%s%3N`\n" >> ~/logs/start_time



# Previous method that opens gnome terminals for each collection method
## Can add a
## ; $SHELL
## to the end of the bash command to keep the terminal open when the task terminates
## Ex:
##	"bash -ic 'rosrun rplidar_ros captureRPLiDAR; #SHELL'"
#
# gnome-terminal -q --tab --title="CSI Camera Data" -e "bash -ic 'rosrun csi_drive_data cam_data_sub.py'"
# gnome-terminal -q --window --title="Server Router" -e "bash -ic 'ssh root@192.168.1.7 eval \"~/collect_data.sh; /bin/sh\"'"
# gnome-terminal -q --window --title="LiDAR Data" -e "bash -ic 'rosrun rplidar_ros captureRPLiDAR'"
# gnome-terminal -q --tab --title="USB Cam 1" -e "bash -ic 'rosrun usb_cam_data <insert_filename_here>'"
# gnome-terminal -q --tab --title="USB Cam 2" -e "bash -ic 'rosrun usb_cam_data <insert_filename_here>'"

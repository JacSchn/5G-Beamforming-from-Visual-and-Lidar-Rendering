#!/bin/bash

# Stop all data collection scripts

echo -e "Stopping at time (ms): `date +%s%3N`\n"
echo -e "Stopping at time (ms): `date +%s%3N`\n" >> ~/logs/collection_end

kill_output(){
	ret_val=$1
	device_name=$2
	if [ $ret_val -ne 0 ]
	then
		echo "Failed to kill $device_name"
	else
		echo "Successfully killed $device_name"
	fi

}

PID=`cat ~/logs/lidar/pid_data`
sudo kill -2 ${PID}
kill_output "$?" "LiDAR data collection"

PID=`cat ~/logs/front_usb/pid_data`
sudo kill -2 ${PID}
kill_output "$?" "Front USB data collection"

PID=`cat ~/logs/rear_usb/pid_data`
sudo kill -2 ${PID}
kill_output "$?" "Rear USB data collection"

sleep 0.5

PID=`cat ~/logs/lidar/pid_launch`
sudo kill -2 ${PID}
kill_output "$?" "LiDAR launch"

PID=`cat ~/logs/front_usb/pid_launch`
sudo kill -2 ${PID}
kill_output "$?" "Front USB launch"

PID=`cat ~/logs/rear_usb/pid_launch`
sudo kill -2 ${PID}
kill_output "$?" "Rear USB launch"


# Stop server and client router data collection
ssh root@192.168.1.7 '~/stop_collect_data.sh'


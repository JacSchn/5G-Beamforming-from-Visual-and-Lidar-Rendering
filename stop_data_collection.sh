#!/bin/bash

# Stop all data collection scripts

echo -e "Stopping at time (ms): `date +%s%3N`\n"
echo -e "Stopping at time (ms): `date +%s%3N`\n" >> ~/logs/collection_end

##########################################
# Output result of killing process
# Arguments:
#	Return val of kill command,
#	Device name
# Returns:
#	0 if successful
#	1 if not successful
##########################################
kill_output(){
	ret_val=$1
	device_name=$2
	if [ $ret_val -ne 0 ]; then
		echo "Failed to kill $device_name"
		retval=1
	else
		echo "Successfully killed $device_name"
		retval=0
	fi
	return "$retval"
}

##########################################
# Forces camera port to be killed
# Arguments:
# 	Port num of camera,
#	Device name
##########################################
force_kill_cam(){
	port=$1
	device_name=$2
	pid=`sudo fuser /dev/video$port`
	err_code=$?

	if [ $err_code == 0 ]; then
		echo "Force stopping $device_name with PID $pid"
		sudo kill -9 $pid
	else
		echo "Cannot kill $device_name on port $port. It does not exist."
	fi
}

# Kill LiDAR data collection
PID=`cat ~/logs/lidar/pid_data`
sudo kill -2 ${PID}
kill_output "$?" "LiDAR data collection"

# Kill front USB data collection
PID=`cat ~/logs/front_usb/pid_data`
sudo kill -2 ${PID}
kill_output "$?" "Front USB data collection"

# Kill rear USB data collection
PID=`cat ~/logs/rear_usb/pid_data`
sudo kill -2 ${PID}
kill_output "$?" "Rear USB data collection"

sleep 0.5

# Kill front USB launch
PID=`cat ~/logs/front_usb/pid_launch`
sudo kill -2 ${PID}
kill_output "$?" "Front USB launch"
ret_val=$?
if [ $ret_val -ne 0 ]; then
	force_kill_cam "0" "front USB"
fi

sleep 0.5

# Kill rear USB launch
PID=`cat ~/logs/rear_usb/pid_launch`
sudo kill -2 ${PID}
kill_output "$?" "Rear USB launch"
ret_val=$?
if [ $ret_val -ne 0 ]; then
	force_kill_cam "2" "rear USB"
fi

# Kill LiDAR launch
PID=`cat ~/logs/lidar/pid_launch`
sudo kill -2 ${PID}
kill_output "$?" "LiDAR launch"

# Stop server and client router data collection
ssh root@192.168.1.7 '~/stop_collect_data.sh'

# Final check to see if USB cameras have been killed
sudo fuser /dev/video0
ret_val=$?
if [ $ret_val -eq 0 ]; then
	force_kill_cam "0" "front USB"
fi

sudo fuser /dev/video2 > /dev/null 2>&1
ret_val=$?
if [ $ret_val -eq 0 ]; then
	force_kill_cam "2" "rear USB"
fi

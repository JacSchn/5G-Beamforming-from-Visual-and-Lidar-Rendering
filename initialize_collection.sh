#!/bin/bash

# Script launchs necessary files prior to collecting data
# Opens each in their own terminal
# This DOES NOT start any data collection

# Sleep time between calls
nap=5

echo "Launching necessary scripts prior to data collection..."
sleep 0.5
echo -e "Checking Nano's Time Sync...\n"
sleep 1
service chrony status
sleep 2

echo -e "\nDate on Nano:" `date`
echo -e "Stop Script if date is incorrect\n"
sleep $nap

# Launch Server Router Terminal
gnome-terminal -q --window --title="Server Router" -e "bash -ic 'ssh root@192.168.1.7; $SHELL'"
ssh root@192.168.1.7 '~/sync_time.sh'
sleep $nap

# Sync Time
echo -e "To access client router, use\n\t$ ssh root@192.168.100.10\nor:\n\t$ ./ssh_client.sh\nMust be connected to server router via ssh first\n"

#Launch Client Router Terminal
gnome-terminal -q --window --title="Client Router" -e "bash -ic 'ssh root@192.168.1.7; $SHELL'"
sleep $nap

# Launch LiDAR
#gnome-terminal -q --window --title="LiDAR Launch" -e "bash -ic 'roslaunch rplidar_ros rplidar_a3.launch; $SHELL'"
#sleep $nap

# Launch Donkeycar
#gnome-terminal -q --window --title="Donkeycar Self Drive" -e "bash -ic 'rosrun csi_drive_data manage.py drive; $SHELL'"

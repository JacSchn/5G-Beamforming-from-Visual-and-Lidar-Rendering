#!/bin/bash

# Script launchs necessary files prior to collecting data
# Opens each in their own terminal
# This DOES NOT start any data collection

## Can add a
## ; $SHELL
## to the end of the bash command to keep the terminal open when the task terminates
## Ex:
##       "bash -ic 'rosrun rplidar_ros captureRPLiDAR; $SHELL'"

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

# Sync Time
ssh root@192.168.1.7 '~/sync_time.sh'

##Launch Server Router Terminal
##Uncomment if a dedicated terminal for the server router is wanted
#gnome-terminal -q --window --title="Server Router" -e "bash -ic 'ssh root@192.168.1.7; $SHELL'"
#sleep $nap

##Launch Client Router Terminal
##Uncomment if a dedicated terminal for the client router is wanted
#gnome-terminal -q --window --title="Client Router" -e "bash -ic 'ssh -J root@192.168.1.7 root@192.168.100.10; $SHELL'"
#sleep $nap

# Launch Client Router boot_network.sh script
# Script pings 60GHz network
gnome-terminal -q --window --title="Client Router" -e "bash -ic 'ssh -J root@192.168.1.7 root@192.168.100.10 eval \"~/data_collection/boot_network.sh; /bin/sh\"'"
sleep $nap

# Launch LiDAR
gnome-terminal -q --window --title="LiDAR Launch" -e "bash -ic 'roslaunch rplidar_ros rplidar_a3.launch; $SHELL'"
sleep $nap

# Launch Donkeycar
gnome-terminal -q --window --title="Donkeycar Self Drive" -e "bash -ic 'rosrun csi_drive_data manage.py drive; $SHELL'"

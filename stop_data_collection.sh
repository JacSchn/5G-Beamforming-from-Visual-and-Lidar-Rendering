#!/bin/bash

# Stop all data collection scripts

echo -e "Stopping at time (ms): `date +%s%3N`\n"

# User has to hit Ctrl+C for several processes to exit cleanly
echo -e "Hit Ctrl+C in LiDAR, CSI cam, and USB cam terminals"

# Stop server and client router data collection
ssh root@192.168.1.7 '~/stop_collect_data.sh'


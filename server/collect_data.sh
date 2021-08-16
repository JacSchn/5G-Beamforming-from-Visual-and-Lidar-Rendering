#!/bin/sh


#### Starting client router using this script does NOT work currently ####
#### Start client router via a direct ssh and launch its start_collect.sh script ####



# Starts data collection for server and client router
# IMPORTANT: Ensure chrony is active and has sucessfully synched all devices before collecting data!!

# Start Server Router Data Collection
sh ~/data_collection/start_collect.sh

# Start Client Router Data Collection
#ssh -f root@192.168.100.10 '~/data_collection/start_collect.sh; $SHELL'

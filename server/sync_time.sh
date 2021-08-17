#!/bin/sh

# Script syncs chrony for both server and client router
# Script must run before collecting data

# Sync server
chronyc online
sleep 10
chronyc sources

# Sync client
# Runs client local sync_time.sh script
ssh root@192.168.100.10 '~/sync_time.sh'

echo -e "Server Date: `date`"
echo "Client Date: `ssh root@192.168.100.10 'date'`"

echo "
Server and Client Router Times Are Synced

Can check connection with
	$ chronyc sources
"


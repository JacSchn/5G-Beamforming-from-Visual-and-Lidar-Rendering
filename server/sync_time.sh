#!/bin/sh

# Script syncs chrony for both server and client router
# Script must run before collecting data

nap=8

# Sync server
chronyc online
sleep $nap
chronyc sources

# Sync client
# Runs client local sync_time.sh script
ssh root@192.168.100.10 '~/sync_time.sh'
sleep $nap

echo "
Server Date: `date`"
echo "Client Date: `ssh root@192.168.100.10 'date'`"

echo "
Server and Client Router Times Are Synced

Can check connection with
	$ chronyc sources
"


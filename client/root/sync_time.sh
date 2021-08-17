#!/bin/sh

# Syncs time with server router
# Must run before data can be collected

echo -e "\nClient Router Beginning Time Sync\n"

chronyc online
sleep 10
chronyc sources

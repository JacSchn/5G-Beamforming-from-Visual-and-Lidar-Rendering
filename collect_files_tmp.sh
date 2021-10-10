#!/bin/bash

# Script to be used to collect all files in the root directory from both routers.

# Script only works if both routers are connected via ethernet.

echo -e "This script will delete all files saved in the server and client directories on the Nano.\n Proceed with caution.\n"
sleep 1

echo -e "Are both routers connected via ethernet?\nStop script if this is not true...\n"
sleep 5

echo -e "Beginning Server router file collection..."
sleep 0.5

rm -r ./server/
scp -r root@192.168.1.7:~/ ./server/
scp -r root@192.168.1.7:/tmp/server_data/* ./server/server_data/
rm -r ./server/.ssh/

echo -e "Completed Server router file collection\n"
sleep 0.5

echo -e "Beginning Client router file collection...\n"
sleep 0.5

rm -r ./client/
scp -r root@192.168.1.10:~/ ./client/
scp -r root@192.168.1.10:/tmp/client_data/* ./client/client_data/
rm -r ./client/.ssh/

echo -e "Completed Client router file collection\nEnding script\n"

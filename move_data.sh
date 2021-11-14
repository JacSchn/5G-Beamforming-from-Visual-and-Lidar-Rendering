#!/bin/bash

# Script moves the data for both the server and client router to the data folder

echo -e "Transferring server router data to the data directory\n"
mv ./server/server_data/* ~/data/server_router
echo -e "Successfully moved server router data to the data directory\n"
sleep 1

echo -e "Transferring client router data to the data directory\n"
mv ./client/client_data/* ~/data/client_router
echo -e "Successfully moved client router data to the data directory\n"
sleep 0.5

#!/bin/sh
mkdir -p ~/server_data/sweep_dump_data/
mkdir -p ~/server_data/bf_data/

sh ~/data_collection/bf_collect.sh &
sh ~/data_collection/sweep_dump_collect.sh true &

echo -e "Server Router is now collecting sweep_dump and bf data starting at `date +%s%3N`\n"

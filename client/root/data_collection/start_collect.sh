#!/bin/sh

# Client router script to start data collection

mkdir -p ~/client_data/sweep_dump_data/
mkdir -p ~/client_data/bf_data/

sh ~/data_collection/bf_collect.sh &
sh ~/data_collection/sweep_dump_collect.sh true &

echo -e "Client Router is now collecting sweep_dump and bf data starting at `date +%s%3N`\n"

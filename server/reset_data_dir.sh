#!/bin/sh

# Script resets the folders holding the sweep_dump and bf data

rm -r ~/server_data/bf_data/
rm -r ~/server_data/sweep_dump_data/

echo -e "Deleted Server data folders\n"

# Reset Client router data
ssh root@192.168.100.10 '~/reset_data_dir.sh'

echo -e "Deleted Client data folders\n"

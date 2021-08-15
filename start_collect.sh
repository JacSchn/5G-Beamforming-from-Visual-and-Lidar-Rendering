#!/bin/sh
mkdir -p ~/server_data/sweep_dump_data/
mkdir -p ~/server_data/bf_data/

#sh ./bf_collect.sh &
sh ./sweep_dump_collect.sh false &

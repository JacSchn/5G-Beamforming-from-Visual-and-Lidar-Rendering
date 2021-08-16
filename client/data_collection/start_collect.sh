#!/bin/sh
mkdir -p ~/client_data/sweep_dump_data/
mkdir -p ~/client_data/bf_data/

iperf_num=`ps | grep iperf3 | wc -l`
if [ $iperf_num -eq 4 ]; then
	echo "This is iperf3 server"
else
	iperf3 -c 192.168.100.7 -p 5103 -t 300 &
fi

sh ~/data_collection/bf_collect.sh &
sh ~/data_collection/sweep_dump_collect.sh true &

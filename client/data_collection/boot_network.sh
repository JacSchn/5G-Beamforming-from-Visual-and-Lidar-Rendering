#!/bin/sh

# Script boots up 60GHz ping
# Run before collecting data

iperf_num=`ps | grep iperf3 | wc -l`
if [ $iperf_num -eq 4 ]; then
	echo "This is iperf3 server"
else
# Increase the value of -t for longer ping time
# -i Speed of bandwith information
# --logfile Logs information about connection

	iperf3 -c 192.168.100.7 -p 5103 -t 3600 -i 60 --logfile network_output
fi

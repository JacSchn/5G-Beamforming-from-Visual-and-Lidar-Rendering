#!/bin/sh

iperf_num=`ps | grep iperf3 | wc -l`
if [ $iperf_num -eq 4 ]; then
	echo "This is iperf3 server"
else
	ps | grep iperf3 | grep -v grep | awk '{print $1}' | xargs kill -9
fi

ps | grep bf_collect.sh | grep -v grep | awk '{print $1}' | xargs kill -9
ps | grep sweep_dump_collect.sh | grep -v grep | awk '{print $1}' | xargs kill -9

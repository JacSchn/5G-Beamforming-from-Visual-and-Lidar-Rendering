#!/bin/sh

# Client router script to stop 60GHz ping and data collection

iperf_num=`ps | grep iperf3 | wc -l`
if [ $iperf_num -eq 4 ]; then
	echo "This is iperf3 server"
else
	ps | grep iperf3 | grep -v grep | awk '{print $1}' | xargs kill -9
fi

echo -e "Killed iperf3 process at `date +%s%3N`\n"

ps | grep bf_collect.sh | grep -v grep | awk '{print $1}' | xargs kill -9
ps | grep sweep_dump_collect.sh | grep -v grep | awk '{print $1}' | xargs kill -9

echo -e "Killed Client Router bf and sweep_dump collection scripts at `date +%s%3N`\n"

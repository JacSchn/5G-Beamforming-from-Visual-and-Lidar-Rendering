#!/bin/sh

# Creates a new file for each bf collection


interval=0.2
interval_compensate=0.05

file_counter=0
filepath=~/server_data/bf_data/"${file_counter}_bf"

filepath=/tmp/server_data/bf_data/"${file_counter}_bf"

file_namer () {
	file_counter=`expr ${file_counter} + 1`
	filepath=/tmp/server_data/bf_data/"${file_counter}_bf"
#	filepath=~/server_data/bf_data/"${file_counter}_bf"
}



while true
do
	timestamp=`date +%s%3N`
	if [ ! -x "$filepath" ]; then
		touch "$filepath"
	fi

	echo "Epochtime(ms): "$timestamp > "$filepath"
	cat /sys/kernel/debug/ieee80211/phy2/wil6210/bf >> "$filepath"
	end_timestamp=`date +%s%3N`
	echo "End_timestamp(ms): "$end_timestamp >> "$filepath"
	exec_time=`expr $end_timestamp - $timestamp`
	echo "Exectime(ms): "$exec_time >> "$filepath"

	file_namer

	sleep $interval_compensate
done

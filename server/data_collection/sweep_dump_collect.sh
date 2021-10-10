#!/bin/sh

# Separate files for each sweep_dump method
# Con: Eats away at storage faster
# Pro: Easier to remove bad data if needed
# Can run for 6+ hrs without running out of space

interval=0.001
swp_counter=`cat /sys/kernel/debug/ieee80211/phy2/wil6210/sweep_dump | grep Counter | awk '{print $2}'`

file_counter=0
filepath=~/server_data/sweep_dump_data/"${file_counter}_sw_d"

# For tmp directory
filepath=/tmp/server_data/sweep_dump_data/"${file_counter}_sw_d"

file_namer () {
	file_counter=`expr ${file_counter} + 1`
	filepath=/tmp/server_data/sweep_dump_data/"${file_counter}_sw_d"
#	filepath=~/server_data/sweep_dump_data/"${file_counter}_sw_d"
}

strip_data () {
	while true
	do
		timestamp=`date +%s%3N`
		if [ ! -x "$filepath" ]; then
			touch "$filepath"
		fi

		curr_swp_count=`cat /sys/kernel/debug/ieee80211/phy2/wil6210/sweep_dump | grep Counter | awk '{print $2}'`

		if [ $swp_counter -gt $curr_swp_count ]; then
			sleep $interval
			continue
		fi

		echo ${timestamp} > $filepath
		cat /sys/kernel/debug/ieee80211/phy2/wil6210/sweep_dump | sed -e 's/\[//' | grep -w "${swp_counter} src:" | sed -n -e 's/\(src: \)//' -n -e 's/\]//' -n -e 's/\(sec: \)//' -n -e 's/\(cdown: \)//' -n -e 's/\(dir: \)//' -n -e 's/\(snr:  \)//' -n -e 's/\(dB \)//p' >> $filepath

		file_namer
		swp_counter=`expr ${swp_counter} + 1`

		sleep $interval
	done
}


default_data () {
	while true
	do
		timestamp=`date +%s%3N`
		if [ ! -x "$filepath" ]; then
			touch "$filepath"
		fi

		curr_swp_count=`cat /sys/kernel/debug/ieee80211/phy2/wil6210/sweep_dump | grep Counter | awk '{print $2}'`
	
		if [ $swp_counter -gt $curr_swp_count ]; then
			sleep $interval
			continue
		fi

		echo ${timestamp} > $filepath	
		cat /sys/kernel/debug/ieee80211/phy2/wil6210/sweep_dump | sed -n -e 's/\(\[\)//p' | grep -w "${swp_counter} src:" >> $filepath

		file_namer
		swp_counter=`expr ${swp_counter} + 1`

		sleep $interval
	done
}

# If true, then stripped data format
# If false, then default data format
if [ $1 ]; then
	strip_data
else 
	default_data
fi

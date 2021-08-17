#!/bin/sh

# File tests the validity of the sweep_dump_collection.sh file
# Test opens each file and checks the sweep_count number
# This number should increment by one from file to file
# If it does not, then there is an issue in the collection method.


# Get number of files
# Substract one for for loop. It includes the last number
file_cnt=`ls ~/client_data/sweep_dump_data/ | wc -l`
echo $file_cnt
file_cnt=`expr ${file_cnt} - 1`

init_num=`cat ~/client_data/sweep_dump_data/0_sw_d | awk '{print $1}'`
init_num=`echo ${init_num} | awk '{print $2}'`
echo "Init sweep_dump count: ${init_num}"

success=true

for i in `seq 0 $file_cnt`
do
	data_amt=`cat ~/client_data/sweep_dump_data/"${i}_sw_d" | awk '{print $1}'`
#	echo $data_amt | awk '{print $2}'
	sweep_num=`echo ${data_amt} | awk '{print $2}'`
	
####### Error if the sweep_num does not equal the incremented initial_num ######
	if [ "$sweep_num" -ne "$init_num" ]; then
		echo "INCORRECT SWEEP_DUMP NUMBER: "
		echo "Init_num= ${init_num}"
		echo "Sweep_num= ${sweep_num}"
		success=false
		break
	fi
	echo $data_amt
#	num_data=`echo ${data_amt} | grep -m 40 -cw ${sweep_num}`
#	echo $num_data


	init_num=`expr ${init_num} + 1`
done

if [ $success ]; then
	echo "sweep_dump numbers are CORRECT"
fi

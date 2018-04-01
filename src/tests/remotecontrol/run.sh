#!/bin/bash

DIR=/home/ubuntu/PROJECTS/HEX/TESTS/remotecontrol

# clear previous log file
rm -f ${DIR}/log

tries=1
running=0
done=0


while [ ${done} -ne 1 ]; do
	# launch program in background
	${DIR}/remote > ${DIR}/log &
	tries=$((${tries}+1))
	# look to see if the program is running
	# this command lists background jobs, filters for ones that are running, then counts them
	# if the program is running, returns 1, else returns 0
	running=`jobs | grep Running | wc | awk '{print $1}'`
	# wait until it disappears
	while [ ${running} -eq 1 ]; do
		sleep 1
		running=`jobs | grep Running | wc | awk '{print $1}'`
	done
	# if we've tried too many times, stop
	if [ ${tries} -gt 25 ]
	then
		done=1
	fi
done

exit 0

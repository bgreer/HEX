#!/bin/bash

DIR=/home/ubuntu/PROJECTS/HEX/TESTS/remotecontrol

rm -f ${DIR}/log

tries=1
running=0
done=0

rm -f ${DIR}/log
while [ ${done} -ne 1 ]; do
	${DIR}/remote > ${DIR}/log &
	tries=$((${tries}+1))
	# wait for exit
	running=`jobs | grep remote | wc | awk '{print $1}'`
	while [ ${running} -eq 1 ]; do
		sleep 1
		running=`jobs | grep Running | wc | awk '{print $1}'`
	done
	if [ ${tries} -gt 25 ]
	then
		done=1
	fi
done

exit 0

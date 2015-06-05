#!/bin/bash

# this script runs on boot
# make damn sure it doesn't crash
# if it does, you'll need to reboot the whole robot
# :(


# the goal here is to run the main code and watch for completion
# if it crashes, rerun it

DIR=/home/bgreer/PROJECTS/HEX/

# clear previous log file
rm -f ${DIR}/runlog

tries=0
running=0
done=0

# TODO: set run.sh LED on

while [ ${done} -ne 1 ]; do
	# launch program in background
	tries=$((${tries}+1))
	${DIR}/hex >> ${DIR}/runlog
	# #? gives the return code, $! gives the pid
	ret=$?
	pid=$!
	echo "Try ${tries} (PID=${pid}) returned exit code ${ret}" >> ${DIR}/runlog

	# if success, we are done!
	if [ ${ret} -eq 0 ]
	then
		done=1
	fi

	# if we've tried too many times, stop
	if [ ${tries} -gt 25 ]
	then
		done=1
	fi
done

#TODO: set run.sh LED off

exit 0

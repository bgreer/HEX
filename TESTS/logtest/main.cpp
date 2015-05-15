#include <iostream>
#include <string>
#include <stdio.h>
#include <thread>
#include <cmath>
#include <iomanip>
#include "/home/bgreer/PROJECTS/HEX/LIB_LOGGER/logger.h"

using namespace std;


int main(void)
{
	int ii, ij;
	float val;
	data_chunk *d;
	logger log;
	double time0;
	struct timeval tv;

	gettimeofday(&tv, NULL);
	time0 = (tv.tv_sec) + tv.tv_usec*1e-6;

	log.init("logfile", true);

	for (ii=0; ii<100; ii++)
	{
		d = new data_chunk('T', time0); // use time relative to program start
		d->add((float)ii);
		val = 0.0;
		for (ij=0; ij<100000; ij++)
			val += (rand()/((float)RAND_MAX))-0.5;
		d->add(val);
		log.send(d);
	}

	log.close();

}

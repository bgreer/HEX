#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <fstream>
#include <vector>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <sys/time.h>
#include <ctime>
#include <thread>
#include <mutex>

#define QUEUE_SIZE 1024

using namespace std;

class data_chunk
{
public:
	unsigned char tag;
	double time, inittime;
	int num; // how many floats of data we have loaded
	int space; // how much space we have allocated
	float *data;
	
	data_chunk (unsigned char t, double time0 = 0.0)
	{
		num = 0;
		space = 1;
		data = new float [space];
		tag = t;
		inittime = time0;
		updateTime();
	}

	void add (float val)
	{
		float *newbuf;
		int ii;
		if (num < space)
		{
			data[num] = val;
		} else {
			space = space*2;
			newbuf = new float [space];
			for (ii=0; ii<num; ii++)
				newbuf[ii] = data[ii];
			delete [] data;
			data = newbuf;
			data[num] = val;
		}
		num ++;
	}

	void updateTime()
	{
		struct timeval tv;
		gettimeofday(&tv, NULL);
		time = (tv.tv_sec) + tv.tv_usec*1e-6 - inittime;
	}

	~data_chunk ()
	{
		delete [] data;
	}

};

class logger
{
public:
	bool initialized, listening;
	bool plaintext;
	ofstream file;
	thread listener;
	// circular buffers
	// queue_start is the index of the first element
	// queue_end is the place to put a new element.
	int queue_start, queue_end, queue_num;
	data_chunk *parcel[QUEUE_SIZE];
	mutex queue_mutex;

	logger();
	void init(char *filename, bool plaintext = false);
	void close();
	void send(data_chunk *d); // logger loop is responsible for freeing memory from chunk
};


void logger_loop (logger *log);

#endif

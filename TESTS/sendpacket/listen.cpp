#include <iostream>
#include <string>
#include <stdio.h>
#include <thread>
#include <cmath>
#include <iomanip>
#include "/home/bgreer/PROJECTS/HEX/LIB_SERIAL/serial.h"

using namespace std;

#define SIZE 128

double getTime()
{
	double ret;
	struct timeval tv;
	gettimeofday(&tv, NULL);
	ret = (tv.tv_sec-1422700000) + tv.tv_usec*1e-6;
	return ret;
}

int main(void)
{
	int ii, ij, ik, ix, iy, size, ind;
	int cx, cy;
	serial ser;
	packet *pack, *pack2, *pack_ask, *pack_data;
	int dsize, psize;
	unsigned char chk;

	// UDOO to Due is /dev/ttymxc3
	ser.init("/dev/ttymxc3", true);
	// pass messages back and forth as fast as possible
	cout << "Program Start." << endl;

	while (1)
	{
		usleep(5000);
	}
	ser.close();
}

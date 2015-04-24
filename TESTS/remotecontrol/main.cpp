#include <iostream>
#include <string>
#include <stdio.h>
#include <thread>
#include <cmath>
#include <iomanip>
#include "/home/bgreer/PROJECTS/HEX/LIB_SERIAL/serial.h"
#include "/home/bgreer/PROJECTS/HEX/LIB_HEXAPOD/hexapod.h"

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
	hexapod hex;
	int dsize, psize;
	double time, lasttime, dt;
	float pos, avgtemp;
	unsigned char chk;

	// UDOO to Due is /dev/ttymxc3
	ser.init_old("/dev/ttymxc3", false);
	// before continuing, ask scontroller for servo data
	// mostly to make sure it's ready to do stuff
	cout << "Confirming Connection.." << endl;
	pack = new packet(16, 'D', 128); // reasonable size?
	pack->data[0] = 1;
	ser.send(pack, true);
	sleep(1);
	
	pack2 = NULL;
	while ((pack2 = ser.recv('E', false)) == NULL)
	{
		ser.send(pack, true);
		sleep(1);
	}
	delete pack;
	cout << "System Ready." << endl;
	for (ii=0; ii<18; ii++)
	{
		memcpy(&pos, pack2->data+1+ii*sizeof(float), 
				sizeof(float));
		hex.servoangle[ii] = pos;
	}
	hex.setAngles();
	delete pack2;


	cout << "Angles read." << endl;

	// sit / stand
	hex.stand();
	dsize = 100;
	psize = SIZE;
	pack = new packet(dsize, 'S', psize);
	for (ii=0; ii<18; ii++)
	{
		pos = hex.servoangle[ii];
		memcpy(pack->data+ii*sizeof(float),
				 &pos, sizeof(float));
	}
	ser.send(pack);

	// max useable speed is 2.0 -> 1 foot per second
	hex.speed = 0.4; // in cycles per second
	hex.turning = -1.0; // [-1,1], rotation in z-axis


	// get ready to ask for data
	pack_ask = new packet(16, 'D', 128);
	pack_ask->data[0] = 5;
	pack_data = NULL;
	ser.send(pack_ask);
	
	cout << "Begin IK" << endl;
	// IK test
	time = 0.0;
	lasttime = getTime();
	while (time < 120.0)
	{
		dt = (getTime() - lasttime);
		time += dt;
		lasttime = getTime();
		hex.step(dt);
		// package positions
		for (ii=0; ii<18; ii++)
		{
			pos = hex.servoangle[ii];
			memcpy(pack->data+(ii)*sizeof(float), 
					&pos, sizeof(float));
		}
		ser.send(pack);
		usleep(20*1000);
		// ask for data?
		if ((pack2=ser.recv('E',false)) != NULL)
		{
			avgtemp = 0.0;
			for (ii=0; ii<18; ii++)
			{
				memcpy(&pos, pack2->data+1+ii*sizeof(float), 
						sizeof(float));
				avgtemp += pos;
			}
			avgtemp /= 18.;
			cout << time << " " << avgtemp << endl;
			delete pack2;
			pack2 = NULL;
			ser.send(pack_ask);
		}
	}

	delete pack;
	ser.close();

}

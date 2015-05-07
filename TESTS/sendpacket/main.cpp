#include <iostream>
#include <string>
#include <stdio.h>
#include <thread>
#include <cmath>
#include <iomanip>
#include "/home/bgreer/PROJECTS/HEX/LIB_SERIAL/serial.h"
#include "/home/bgreer/PROJECTS/HEX/LIB_HEXAPOD/hexapod.h"

using namespace std;

#define TURN_TOL 0.05
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
	float pos, target[3], avgtemp;
	float phase, xpos, zpos, fdf;
	double time, dt, lasttime, modtime, modtime2, speed, lastdata;
	float leftsweep, rightsweep, turning;
	unsigned char chk;

	// UDOO to Due is /dev/ttymxc3
	ser.init_old("/dev/ttyUSB0", false);
	// before continuing, ask scontroller for servo data
	// mostly to make sure it's ready to do stuff
	cout << "Confirming Connection.." << endl;
	pack = new packet(16, 'A'); // reasonable size?
	pack->data[0] = 0x05; // request data
	pack->data[1] = 0x01; // i want positions
	pack->data[2] = 'D'; // send back to me
	ser.send(pack, true);
	sleep(1);
	
	pack2 = NULL;
	while ((pack2 = ser.recv('D', false)) == NULL)
	{
		ser.send(pack, true);
		sleep(1);
	}
	delete pack;
	cout << "System Ready." << endl;
	for (ii=0; ii<18; ii++)
	{
		memcpy(&pos, pack2->data+1+ii*(sizeof(float)+sizeof(uint8_t)), 
				sizeof(float));
		hex.servoangle[ii] = pos;
		cout << pos << endl;
	}
	hex.setAngles();
	delete pack2;

	cout << "Angles read." << endl;

	// sit / stand
	hex.stand();
	dsize = 100;
	psize = SIZE;
	pack = new packet(dsize, 'A', psize);
	pack->data[0] = 0x01; // set servo positions
	for (ii=0; ii<18; ii++)
	{
		pos = hex.servoangle[ii];
		memcpy(pack->data+1+ii*sizeof(float),
				 &pos, sizeof(float));
	}
	ser.send(pack);


	// max useable speed is 2.0 -> 1 foot per second
	speed = 0.1; // in cycles per second
	fdf = 0.55; // foot-down fraction
	// differential sweep is where legs on one side step farther than the other side
	// this allows for turning.
	// the max sweep should be 5x what the bezier curve gives in the x-direction
	// leftsweep,rightsweep should be [-1,1]
	// one of them should always be maxed out, any lower is just a slower walking speed
	turning = 0.0; // [-1,1], rotation in z-axis


	// get ready to ask for data
	pack_ask = new packet(16, 'A');
	pack_ask->data[0] = 0x05; // request data
	pack_ask->data[1] = 0x02; // i want temperature
	pack_ask->data[2] = 'D'; // send back to me
	pack_data = NULL;
	ser.send(pack_ask);
	
	cout << "Begin IK" << endl;
	// IK test
	time = 0.0;
	modtime = 0.0;
	lasttime = getTime();
	lastdata = lasttime;
	while (time < 120.0)
	{
		for (ik=0; ik<6; ik++)
		{

			// x position
			if (ik < 3) target[0] = hex.legpos[ik][0];
			else target[0] = hex.legpos[ik][0];
			// y position
			if (ik < 3) target[1] = 19.;
			else target[1] = -18.;
			if (ik == 1) target[1] = 20.0;
			if (ik == 4) target[1] = -07.0;
			// z position
			target[2] = -10.0;


			if (hex.IKSolve(ik,target))
			{
				hex.setServoAngles();
				// package positions
				for (ii=0; ii<3; ii++)
				{
					pos = hex.servoangle[ik*3+ii];
					memcpy(pack->data+1+(ik*3+ii)*sizeof(float), 
							&pos, sizeof(float));
				}
			}
		}
		ser.send(pack);
		usleep(20*1000);
		// ask for data?
		if (getTime()-lastdata > 1.0)
		{
		if ((pack2=ser.recv('D',false)) != NULL)
		{
			avgtemp = 0.0;
			for (ii=0; ii<18; ii++)
			{
				memcpy(&pos, pack2->data+1+ii*(sizeof(float)+sizeof(uint8_t)), 
						sizeof(float));
				avgtemp += pos;
			}
			avgtemp /= 18.;
			cout << time << " " << avgtemp << endl;
			delete pack2;
			pack2 = NULL;
			lastdata = getTime();
			ser.send(pack_ask);
		}
		}

		dt = (getTime() - lasttime);
		lasttime = getTime();
		time += dt;
		modtime += dt*speed;
		if (modtime > 1.0) modtime -= 1.0;
		if (modtime < 0.0) modtime += 1.0;
	}

	delete pack;
	ser.close();

}

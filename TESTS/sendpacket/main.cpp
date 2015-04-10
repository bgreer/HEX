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
	float pos, target[3], avgtemp;
	float phase, xpos, zpos, fdf;
	double time, dt, lasttime, modtime, modtime2, speed;
	unsigned char chk;

	ser.init_old("/dev/ttyUSB0", false);
	// before continuing, ask scontroller for servo data
	// mostly to make sure it's ready to do stuff
	cout << "Confirming Connection.." << endl;
	pack = new packet(16, 'D', 128); // reasonable size?
	pack->buffer[PACKET_HEADER_SIZE] = 1;
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
		memcpy(&pos, pack2->buffer+PACKET_HEADER_SIZE+1+ii*sizeof(float), 
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
		memcpy(pack->buffer+PACKET_HEADER_SIZE+ii*sizeof(float),
				 &pos, sizeof(float));
	}
	pack->buffer[psize-1] = '\n';
	ser.send(pack);

	// max useable speed is 2.0 -> 1 foot per second
	speed = 1.0; // in cycles per second
	fdf = 0.55; // foot-down fraction

	// get ready to ask for data
	pack_ask = new packet(16, 'D', 128);
	pack_ask->buffer[PACKET_HEADER_SIZE] = 2;
	pack_data = NULL;
	ser.send(pack_ask);
	
	cout << "Begin IK" << endl;
	// IK test
	time = 0.0;
	modtime = 0.0;
	lasttime = getTime();
	while (time < 120.0)
	{
		for (ik=0; ik<6; ik++)
		{
			phase = (3.14159)*ik;
			// set target relative to leg
			modtime2 = fmod(modtime+0.5*ik+(hex.legpos[ik][0]-hex.legpos[0][0])*0.0125, 1.0);
			if (modtime2 < fdf) hex.b2d_walk_down.getPos(modtime2/fdf, &xpos, &zpos);
			else hex.b2d_walk_up.getPos((modtime2-fdf)/(1.-fdf), &xpos, &zpos);

			target[0] = hex.legpos[ik][0]*1.5 + 5.0*xpos;
			if (ik < 3) target[1] = 14.0;
			else target[1] = -14.0;
			if (ik == 1) target[1] = 16.0;
			if (ik == 4) target[1] = -16.0;
			target[2] = -11.0 + zpos*2.0;

			//target[2] += 2.0*sin(time*4);


			if (hex.IKSolve(ik,target))
			{
				hex.setServoAngles();
				// package positions
				for (ii=0; ii<3; ii++)
				{
					pos = hex.servoangle[ik*3+ii];
					memcpy(pack->buffer+PACKET_HEADER_SIZE+(ik*3+ii)*sizeof(float), 
							&pos, sizeof(float));
				}
			}
		}
		pack->buffer[psize-1] = '\n';
		ser.send(pack);
		usleep(20*1000);
		// ask for data?
		if ((pack2=ser.recv('E',false)) != NULL)
		{
			avgtemp = 0.0;
			for (ii=0; ii<18; ii++)
			{
				memcpy(&pos, pack2->buffer+PACKET_HEADER_SIZE+1+ii*sizeof(float), 
						sizeof(float));
				avgtemp += pos;
			}
			avgtemp /= 18.;
			cout << time << " " << avgtemp << endl;
			delete pack2;
			pack2 = NULL;
			ser.send(pack_ask);
		}

		dt = (getTime() - lasttime);
		lasttime = getTime();
		time += dt;
		modtime += dt*speed;
		if (modtime > 1.0) modtime -= 1.0;
		if (modtime < 0.0) modtime += 1.0;
	}

/*
	for (ii=0; ii<500; ii++)
	{
		target[0] = 100.0*cos(2.*3.14159*ii/300.);
		target[1] = 100.*sin(2.*3.14159*ii/300.);
		target[2] = 0.0;
		ind = PACKET_HEADER_SIZE;
		for (ij=0; ij<6; ij++)
		{
			hex.IKSolve(ij,target);
			if (true)
			{
				pos = hex.angle[ij*3+0];
				cout << pos << " ";
				memcpy(pack->buffer+ind, &pos, sizeof(float));
				ind += sizeof(float);
				pos = hex.angle[ij*3+1];
				cout << pos << " ";
				memcpy(pack->buffer+ind, &pos, sizeof(float));
				ind += sizeof(float);
				pos = hex.angle[ij*3+2];
				cout << pos << " ";
				memcpy(pack->buffer+ind, &pos, sizeof(float));
				ind += sizeof(float);
			}
		}
			cout << endl;
		
		for (ij=0; ij<18; ij++)
		{
			pos = 150.0 + 20.*sin(2.*3.14159*(ii/100. + ij/6.));
			//memcpy(pack->buffer+PACKET_HEADER_SIZE+ij*sizeof(float),&pos, sizeof(float));
		}
		
		pack->buffer[psize-1] = '\n';

		ser.send(pack);
		usleep(20*1000);
	}
	*/

	delete pack;
	ser.close();

}

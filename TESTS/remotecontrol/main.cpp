#include <iostream>
#include <string>
#include <stdio.h>
#include <thread>
#include <cmath>
#include <iomanip>
#include <SDL.h>
#include "/home/bgreer/PROJECTS/HEX/LIB_SERIAL/serial.h"
#include "/home/bgreer/PROJECTS/HEX/LIB_HEXAPOD/hexapod.h"

/* buttons:
 * 	0 = A - flight mode != landed
 * 		1 = B - flight mode = landed
 * 			2 = X - resend user controls
 * 				3 = Y - cycle flight mode
 * 					4 = LB - thrust mode
 * 						5 = RB - set thrust zero point
 * 							6 = Select - send stats
 * 								7 = Start - arm motors
 * 									8 = XBOX - kill switch
 * 										9 = Left Axis
 * 											10 = Right Axis
 * 											*/
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
	SDL_Surface *screen;
	SDL_Event event;
	SDL_Joystick *joy;
	int dsize, psize;
	double time, lasttime, dt;
	float pos, avgtemp, joyval;
	unsigned char chk;
	bool cont, quit;

	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK) < 0)
	{
		cout << "ERROR in SDL init: " << SDL_GetError() << endl;
		return -1;
	}
	atexit(SDL_Quit);
	screen = SDL_SetVideoMode(800, 480, 32, SDL_NOFRAME | SDL_FULLSCREEN);
	SDL_ShowCursor(0);
	SDL_JoystickEventState(SDL_ENABLE);
	// sort out joystick control
	if (SDL_NumJoysticks()>0)
	{
		joy=SDL_JoystickOpen(0);
		if (joy)
		{
			cout << "Joystick Connected." << endl;
		} else {
			cout << "ERROR: could not connect to joystick!" << endl;
			return -1;
		}
	} else {
		cout << "ERROR: could not find joystick!" << endl;
		return -1;
	}

	// UDOO to Due is /dev/ttymxc3
	ser.init_old("/dev/ttymxc3", false);
	
	cout << "Press Start to connect." << endl;
	cont = false;
	while (!cont)
	{
		while (SDL_PollEvent(&event))
		{
			switch (event.type)
			{
				case SDL_JOYBUTTONDOWN:
					if (event.jbutton.button == 7) cont = true;
					break;
			}
		}
		SDL_Delay(10);
	}
	
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
	dsize = 100;
	psize = SIZE;
	pack = new packet(dsize, 'S', psize);

	// max useable speed is 2.0 -> 1 foot per second
	hex.speed = 0.0; // in cycles per second
	hex.turning = 0.0; // [-1,1], rotation in z-axis


	// get ready to ask for data
	pack_ask = new packet(16, 'D', 128);
	pack_ask->data[0] = 5;
	pack_data = NULL;
	ser.send(pack_ask);
	
	cout << "Begin IK" << endl;
	// IK test
	time = 0.0;
	lasttime = getTime();
	hex.safeStand();
	while (hex.ssrunning)
	{
		dt = (getTime() - lasttime);
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

	}
	lasttime = getTime();
	quit = false;
	while (!quit)
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
		// ask for data?
		/*
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
		*/
		while (SDL_PollEvent(&event))
		{
			switch (event.type)
			{
				case SDL_JOYBUTTONDOWN:
					if (event.jbutton.button == 7) quit = true;
					break;
				case SDL_JOYAXISMOTION:
					joyval = event.jaxis.value/32767.;
					if (event.jaxis.axis == 1)
					{
						if (joyval > 0.1) hex.speed = joyval - 0.1;
						else if (joyval < -0.1) hex.speed = joyval + 0.1;
						else hex.speed = 0.0;
					}
					if (event.jaxis.axis == 2)
					{
						if (joyval > 0.1) hex.turning = (joyval-0.1)*0.25;
						else if (joyval < -0.1) hex.turning = (joyval+0.1)*0.25;
						else hex.turning = 0.0;
					}
					break;
			}
		}
		SDL_Delay(20);
	}

	cout << "Quitting.." << endl;

	delete pack;
	ser.close();

}

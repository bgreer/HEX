#include <iostream>
#include <string>
#include <stdio.h>
#include <thread>
#include <cmath>
#include <iomanip>
#include <SDL.h>
#include "/home/bgreer/PROJECTS/HEX/LIB_SERIAL/serial.h"
#include "/home/bgreer/PROJECTS/HEX/LIB_HEXAPOD/hexapod.h"

/* xboxdrv button ids for xbox 360 controller:
 * 0 = A
 * 1 = B
 * 2 = X
 * 3 = Y
 * 4 = LB
 * 5 = RB
 * 6 = Select
 * 7 = Start
 * 8 = XBOX
 * 9 = Left Stick Click
 * 10 = Right Stick Click
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
	int cx, cy, attempts;
	serial ser;
	packet *pack, *pack2, *pack_ask, *pack_data;
	hexapod hex;
	SDL_Surface *screen;
	SDL_Event event;
	SDL_Joystick *joy;
	int dsize, psize;
	double time, lasttime, dt, lastdata;
	uint8_t errcode;
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
	// step 1: create a packet destined for the arbotix-m
	pack = new packet(16, 'A'); // reasonable size?
	// step 2: set command byte to 0x05, request for data
	pack->data[0] = 0x05;
	// step 3: set request byte to 0x01, requesting servo angles
	pack->data[1] = 0x01;
	// step 4: set return byte to 'D', send back to Due
	pack->data[2] = 'D';
	// step 5: send the packet
	ser.send(pack, true);
	sleep(1);
	
	// the Arbotix-M might be off or initializing, so keep sending the request
	// until you hear something back
	pack2 = NULL;
	while ((pack2 = ser.recv('D', false)) == NULL)
	{
		ser.send(pack, true);
		sleep(1);
	}
	delete pack;
	
	// we received data from the Arbotix-M, it's good to continue
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
	pack = new packet(dsize, 'A', psize);
	pack->data[0] = 0x01; // set servo positions

	// max useable speed is 2.0 -> 1 foot per second
	hex.speed = 0.0; // in cycles per second
	hex.turning = 0.0; // [-1,1], rotation in z-axis


	// get ready to ask for data
	pack_ask = new packet(16, 'A');
	pack_ask->data[0] = 0x05;
	pack_ask->data[1] = 0x02; // want temperature
	pack_ask->data[2] = 'U';
	pack_data = NULL;
	ser.send(pack_ask);
	
	cout << "Begin IK" << endl;
	// IK test
	time = 0.0;
	lasttime = getTime();
	lastdata = lasttime;
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
			memcpy(pack->data+1+(ii)*(sizeof(float)+sizeof(uint8_t)), 
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
			memcpy(pack->data+1+(ii)*sizeof(float), 
					&pos, sizeof(float));
		}
		ser.send(pack);
		// ask for data?
		
		if (getTime() - lastdata > 1.0)
		{
		if ((pack2=ser.recv('U',false)) != NULL)
		{
			avgtemp = 0.0;
			for (ii=0; ii<18; ii++)
			{
				memcpy(&pos, pack2->data+1+ii*(sizeof(float)+sizeof(uint8_t)), 
						sizeof(float));
				memcpy(&errcode, pack2->data+1+ii*(sizeof(float)+sizeof(uint8_t))+sizeof(float), sizeof(uint8_t));
				avgtemp += pos;
			}
			avgtemp /= 18.;
			cout << time << " " << avgtemp << endl;
			delete pack2;
			lastdata = getTime();
			pack2 = NULL;
			ser.send(pack_ask);
		}
		}
		
		while (SDL_PollEvent(&event))
		{
			switch (event.type)
			{
				case SDL_JOYBUTTONDOWN:
					if (event.jbutton.button == 7) quit = true;
					break;
				case SDL_JOYAXISMOTION:
					joyval = event.jaxis.value/32767.;
					if (event.jaxis.axis == 1) // L stick, yaxis
					{
						if (joyval > 0.1) hex.speed = joyval - 0.1;
						else if (joyval < -0.1) hex.speed = joyval + 0.1;
						else hex.speed = 0.0;
					}
					if (event.jaxis.axis == 2) // R stick, xaxis
					{
						if (joyval > 0.1) hex.turning = (joyval-0.1);
						else if (joyval < -0.1) hex.turning = (joyval+0.1);
						else hex.turning = 0.0;
					}
					if (event.jaxis.axis == 3) // R stick, yaxis
					{
						if (joyval > 0.1) hex.standheight = (joyval-0.1)*2.0;
						else if (joyval < -0.1) hex.standheight = (joyval+0.1)*2.0;
						else hex.standheight = 0.0;
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

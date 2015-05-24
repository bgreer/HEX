#include <iostream>
#include <string>
#include <stdio.h>
#include <thread>
#include <cmath>
#include <iomanip>
#include "header.h"


#define SIZE 128

int main(void)
{
	int ii, ij, ik, ix, iy, size, ind;
	int cx, cy, attempts;
	serial ser;
	packet *pack, *pack2, *pack_ask, *pack_data;
	packet *pack_enable, *pack_disable;
	hexapod hex;
	data_chunk *d;
	logger log;
	SDL_Surface *screen;
	SDL_Joystick *joy;
	int dsize, psize;
	double time, lasttime, dt, lastdata, inittime;
	uint8_t errcode;
	float pos, avgtemp, joyval, maxval;
	unsigned char chk;
	bool quit;

	// begin logging to file
	// this launches a new thread for async logging
	log.init("logfile", true);
	inittime = getTime();

	// set up sdl for joystick usage
	initSDL(screen, joy); // manual.cpp

	// set up serial connection to Due
	// this launches a new thread for serial listening
	ser.init_old("/dev/ttymxc3", false);

	// wait for user to press Start
	cout << "Press Start to connect." << endl;
	getButtonPress(7, true); // blocking wait for Start button, manual.cpp

	// prep the motor enable / disable packets
	pack_enable = new packet(18, 'A');
	pack_enable->data[0] = 0x03;
	pack_disable = new packet(18, 'A');
	pack_disable->data[0] = 0x03;
	for (ii=0; ii<18; ii++)
	{
		pack_enable->data[ii+1] = 0x01;
		pack_disable->data[ii+1] = 0x00;
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
	pack->data[2] = 'U';
	// step 5: send the packet
	ser.send(pack, true);
	sleep(1);
	
	// the Arbotix-M might be off or initializing, so keep sending the request
	// until you hear something back
	pack2 = NULL;
	attempts = 0;
	while ((pack2 = ser.recv('U', false)) == NULL && attempts < 10)
	{
		attempts ++;
		ser.send(pack, true);
		sleep(1);
	}
	delete pack;
	if (attempts == 10)
	{
		cout << "ERROR: could not connect to servo controller!" << endl;
		return -1;
	}
	
	// we received data from the Arbotix-M, it's good to continue
	cout << "System Ready." << endl;
	// let the hex library know where the actual servos are right now
	for (ii=0; ii<18; ii++)
	{
		memcpy(&pos, pack2->data+1+ii*(sizeof(float)+sizeof(uint8_t)), 
				sizeof(float));
		cout << pos << endl;
		hex.servoangle[ii] = pos;
	}
	hex.setAngles();
	delete pack2;


	
	// sit / stand
	dsize = 100;
	psize = SIZE;
	pack = new packet(dsize, 'A', psize);
	pack->data[0] = 0x01; // set servo positions

	// max useable speed is 2.0 -> 1 foot per second
	hex.speed = 0.0; // in cycles per second
	hex.turning = 0.0; // [-1,1], rotation in z-axis

	ser.send(pack_enable);

	// get ready to ask for data
	pack_ask = new packet(16, 'A');
	pack_ask->data[0] = 0x05;
	pack_ask->data[1] = 0x02; // want temperature
	pack_ask->data[2] = 'U';
	pack_data = NULL;

	// servos are enabled, try to stand up safely
	cout << "Performing Safe Stand.." << endl;
	performSafeStand(&hex, &ser);

	// set up timing data
	time = 0.0;
	lasttime = getTime();
	lastdata = lasttime;

	// MAIN LOOP
	quit = false;
	while (!quit)
	{
		// increment time
		dt = (getTime() - lasttime);
		time += dt;
		lasttime = getTime();

		// let hex library update internal variables
		hex.step(dt);
		// send updated servo positions to servo controller
		sendServoPositions(&hex, &ser);

		// log hexlib internal tracking
		d = new data_chunk('P', inittime);
		d->add(hex.dr_ang);
		d->add(hex.dr_xpos);
		d->add(hex.dr_ypos);
		d->add(hex.maxsweep);
		d->add(hex.speedmodifier);
		log.send(d);

		// ask for data from servos
		if (getTime() - lastdata > 1.0)
		{
		if ((pack2=ser.recv('U',false)) != NULL)
		{
			maxval = 1e-10;
			avgtemp = 0.0;
			for (ii=0; ii<18; ii++)
			{
				memcpy(&pos, pack2->data+1+ii*(sizeof(float)+sizeof(uint8_t)), 
						sizeof(float));
				memcpy(&errcode, pack2->data+1+ii*(sizeof(float)+sizeof(uint8_t))+sizeof(float), sizeof(uint8_t));
				avgtemp += pos;
				if (pos > maxval) maxval = pos;
				if (errcode != 0) // 32=overload, 4=temperature, 1=voltage
					cout << "SERVO ERROR: "<< (int)(errcode) << " ON SERVO " << ii << endl;
			}
			avgtemp /= 18.;
			d = new data_chunk('T', inittime);
			d->add(avgtemp);
			log.send(d);
			delete pack2;
			lastdata = getTime();
			pack2 = NULL;
			ser.send(pack_ask);
			usleep(20*1000);
		}
		}
		
		// look for joystick commands
		updateManualControl(&quit, &(hex.speed), &(hex.turning),
				&(hex.standheight));

		// main loop delay
		SDL_Delay(20);
	}

	cout << "Quitting.." << endl;

	// tell the servos to relax
	ser.send(pack_disable);
	delete pack;

	// close serial port
	ser.close();
	
	// stop logging, close file
	log.close();


}

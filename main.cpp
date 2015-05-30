#include <iostream>
#include <string>
#include <stdio.h>
#include <thread>
#include <cmath>
#include <iomanip>
#include "header.h"

using namespace std;

#define SIZE 128

int main(int argc, char *argv[])
{
	int ii, ij, ik, ix, iy, size, ind;
	int cx, cy, attempts, scans;
	// so many custom classes!
	serial ser;
	packet *pack, *pack2, *pack_ask, *pack_data;
	hexapod hex;
	data_chunk *d, *d2;
	logger log;
	scan *lidar_scan;
	slam slammer;
#ifdef MANUAL
	SDL_Surface *screen;
	SDL_Joystick *joy;
#endif
	double time, lasttime, dt, lastdata, inittime;
	double lastslam, lastscan, lastloop;
	uint8_t errcode;
	float pos, avgtemp, joyval, maxval;
	float prevx, prevy, preva; // for slam guessi
	float dx, dy, da;
	unsigned char chk;
	bool quit;
	uint32_t delaytime;

	prevx = 0.0;
	prevy = 0.0;
	preva = 0.0;

	cout << "Initializing SLAM.." << endl;
	slammer.init(192,192,10.0);
	slammer.setRegularization(atof(argv[1]),atof(argv[1]),atof(argv[2]));

	// begin logging to file
	// this launches a new thread for async logging
	log.init("logfile", true);
	inittime = getTime();

	// set up sdl for joystick usage
#ifdef MANUAL
	if (initSDL(screen, joy) != 0) return -1;
#endif

	// set up serial connection to Due
	// this launches a new thread for serial listening
	ser.init_old("/dev/ttymxc3", false);

	// wait for user to press Start
#ifdef MANUAL
	cout << "Press Start to connect." << endl;
	getButtonPress(7, true); // blocking wait for Start button, manual.cpp
#else
	// wait for init button
#endif
	
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
	while ((pack2 = ser.recv('U', 0x01, false)) == NULL && attempts < 10)
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
		hex.servoangle[ii] = pos;
	}
	hex.setAngles();
	delete pack2;

	
	// sit / stand
	pack = new packet(100, 'A');
	pack->data[0] = 0x01; // set servo positions

	// max useable speed is 2.0 -> 1 foot per second
	hex.speed = 0.0; // in cycles per second
	hex.turning = 0.0; // [-1,1], rotation in z-axis

	enableServos(&ser);

	// start up lidar unit
	usleep(50*1000);
	setLIDARSpin(&ser, true);

	// servos are enabled, try to stand up safely
	cout << "Performing Safe Stand.." << endl;
	performSafeStand(&hex, &ser);

	// wait a while for lidar to get up to speed
	usleep(1000*1000*3);
	// get lidar data, integrate slam map
	cout << "Obtaining initial LIDAR map.." << endl;
	scans = 0;
	while (scans < 360)
	{
		if ((lidar_scan=getLIDARData(&ser, true)) != NULL)
		{
			slammer.integrate(lidar_scan, 0.0, 0.0, 0.0);
			delete lidar_scan;
			scans ++;
		}
		usleep(1000*10);
	}
	lastscan = getTime();
	lastslam = getTime();

	// get ready to ask for data
	pack_ask = new packet(16, 'A');
	pack_ask->data[0] = 0x05;
	pack_ask->data[1] = 0x03; // want temperature
	pack_ask->data[2] = 'U';
	pack_data = NULL;


	// set up timing data
	time = 0.0;
	lasttime = getTime();
	lastdata = lasttime;

	// MAIN LOOP
	quit = false;
	cout << "Running main loop." << endl;
	lastloop = getTime();
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

		if (getTime() - lastscan > 0.02)
		{
			usleep(1000);
			if ((lidar_scan=getLIDARData(&ser, true)) != NULL)
			{
				if (getTime() - lastslam > 0.5)
				{
					lastslam = getTime();
					dx = hex.dr_xpos - prevx;
					dy = hex.dr_ypos - prevy;
					da = hex.dr_ang - preva;
					prevx = hex.dr_xpos;
					prevy = hex.dr_ypos;
					preva = hex.dr_ang;
					slammer.submitScan(lidar_scan, 
							slammer.currx+dx, slammer.curry+dy, slammer.currang+da);
				}
				delete lidar_scan;
			}
			lastscan = getTime();
		}
		
		// log hexlib internal tracking
		d = new data_chunk('P', inittime);
		d->add(hex.dr_xpos);
		d->add(hex.dr_ypos);
		d->add(hex.dr_ang);
		log.send(d);
		// log slam tracking
		d2 = new data_chunk('S', inittime);
		d2->add(slammer.currx);
		d2->add(slammer.curry);
		d2->add(slammer.currang);
		log.send(d2);

		// ask for data from servos
		/*
		if (getTime() - lastdata > 1.0)
		{
		if ((pack2=ser.recv('U',0x03,false)) != NULL)
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
		}*/
		
		// look for joystick commands
#ifdef MANUAL
		updateManualControl(&quit, &(hex.speed), &(hex.turning),
				&(hex.standheight));
#endif

		// main loop delay
		delaytime = (uint32_t)min(20000.,20000.-(getTime()-lastloop)*1000.*1000.);
		usleep(delaytime);
		lastloop = getTime();
	}

	cout << "Quitting.." << endl;

	slammer.outputMap("map");
	slammer.close();
	// stop the lidar unit
	setLIDARSpin(&ser, false);

	// tell the servos to relax
	disableServos(&ser);
	delete pack;

	// close serial port
	ser.close();
	
	// stop logging, close file
	log.close();


}

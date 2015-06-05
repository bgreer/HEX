#include <iostream>
#include <string>
#include <stdio.h>
#include <thread>
#include <cmath>
#include <iomanip>
#include "header.h"

using namespace std;

int main(int argc, char *argv[])
{
	int ii, ij, ik, ix, iy, size, ind;
	int cx, cy, attempts, scans;
	// so many custom classes!
	autonav nav;
	serial ser;
	hexapod hex;
	data_chunk *d, *d2;
	logger log;
	scan *lidar_scan;
	slam slammer;
#ifdef MANUAL
	SDL_Surface *screen;
	SDL_Joystick *joy;
#endif
	double time, lasttime, dt, lastdata;
	double lastslam, lastscan, lastloop, lastnav;
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

	// STEP 1: initialize classes
	if (DEBUG) cout << "Initializing..." << endl;
	log.init("logfile", true); // log file
	slammer.init(128,128,5.0);
	slammer.setRegularization(0.3,0.3,1.0);
	nav.init(&hex, &slammer, &log, 0,0,0);
	ser.init_old("/dev/ttymxc3", false); // serial comm with Due
#ifdef MANUAL
	if (initSDL(screen, joy) != 0) return -1;
#endif


	// STEP 2: load waypoints for autonav
	nav.addTarget(100.0, 0.0, 10.0);


	// STEP 3: confirm connection with arbotix-m
	if (DEBUG) cout << "Confirming Connection.." << endl;
	if (!(confirmArbotixConnection(&ser, &hex)))
	{
		if (DEBUG) cout << "ERROR: could not connect to servo controller!" << endl;
		d = new data_chunk('E');
		d->add(1);
		log.send(d);
		return -1;
	}
	if (DEBUG) cout << "System Ready." << endl;


	// wait for user input before moving on
#ifdef MANUAL
	if (DEBUG) cout << "Press Start to continue." << endl;
	getButtonPress(7, true); // blocking wait for Start button, manual.cpp
#else
	// wait for init button
#endif

	// STEP 4: enable hardware
	enableServos(&ser); // tell servos to turn on
	usleep(50*1000); // don't send stuff too quickly..
	setLIDARSpin(&ser, true); // lidar motor spin up
	// stand up safely while LIDAR gets up to speed
	if (DEBUG) cout << "Performing Safe Stand.." << endl;
	performSafeStand(&hex, &ser);
	usleep(1000*1000*3); // wait more for LIDAR

	// wait for user input before using lidar scans
#ifdef MANUAL
	if (DEBUG) cout << "Press Start to continue." << endl;
	getButtonPress(7, true); // blocking wait for Start button, manual.cpp
#else
	// wait for init button
#endif

	// STEP 5: get initial LIDAR scan of surroundings
	// needs a few scans, but allow for user to keep scanning after that
	if (DEBUG) cout << "Obtaining initial LIDAR map.." << endl;
	if (DEBUG) cout << "Press Start to finish scanning." << endl;
	scans = 0;
	quit = false;
	while (scans < 360 && !quit)
	{
		if ((lidar_scan=getLIDARData(&ser, true)) != NULL)
		{
			slammer.integrate(lidar_scan, 0.0, 0.0, 0.0);
			delete lidar_scan;
			scans ++;
		}
#ifdef MANUAL
		quit = getButtonPress(7, false);
#else
		// check status of button
#endif
		usleep(1000*10);
	}
	slammer.filter(); // force filtering so autonav has something to work with
	// set up timing data
	lastscan = getTime();
	lastslam = getTime();
	lastnav = getTime();
	time = 0.0;
	lasttime = getTime();
	lastdata = lasttime;


	// Now we are ready to move around!
	

	// MAIN LOOP
	quit = false;
	if (DEBUG) cout << "Running main loop." << endl;
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

		// get LIDAR scan and update SLAM
		if (getTime() - lastscan > 0.02)
		{
			usleep(1000);
			if ((lidar_scan=getLIDARData(&ser, true)) != NULL)
			{
				if (getTime() - lastslam > 1.5)
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
		d = new data_chunk('P');
		d->add(hex.dr_xpos);
		d->add(hex.dr_ypos);
		d->add(hex.dr_ang);
		log.send(d);
		// log slam tracking
		d2 = new data_chunk('S');
		d2->add(slammer.currx);
		d2->add(slammer.curry);
		d2->add(slammer.currang);
		log.send(d2);

		// main navigation commands:
#ifdef MANUAL
		// look for remote control commands
		updateManualControl(&quit, &(hex.speed), &(hex.turning),
				&(hex.standheight));
#else
		// use autonav to decide motion
		if (getTime() - lastnav > 0.2)
		{
			// use autonav to set hexapod speed and turning
			nav.solve(slammer.currx, slammer.curry, slammer.currang);
			lastnav = getTime();
		}
		// check for end of target list
		nav.anlock.lock();
		if (nav.currtarget == nav.target_x.size())
		{
			performRaceFinish(&hex, &ser);
			quit = true;
		}
		nav.anlock.unlock();
#endif


		// main loop delay
		// try to keep update rate at 1/20ms as best as possible
		delaytime = (uint32_t)min(20000.,20000.-(getTime()-lastloop)*1000.*1000.);
		usleep(delaytime);
		lastloop = getTime();
	}

	// begin clean shutdown
	if (DEBUG) cout << "Quitting.." << endl;
	slammer.outputMap("map"); // save slam map to file
	slammer.close();
	nav.close();
	setLIDARSpin(&ser, false); // tell Due to stop the LIDAR motor
	disableServos(&ser); // set servos to disabled (no torque)
	ser.close(); // close serial port
	log.close(); // finish logging, close log file


}

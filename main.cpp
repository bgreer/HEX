#include <iostream>
#include <string>
#include <stdio.h>
#include <thread>
#include <cmath>
#include <iomanip>
#include "header.h"

using namespace std;

void parseCmdLine(int argc, char *argv[], 
		float *reg_pos, float *reg_ang, int *grid_size, float *grid_scale, 
		string *waypoint_fname, float *init_x, float *init_y, float *init_ang);

int main(int argc, char *argv[])
{
	int ii, scans;
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
	float prevx, prevy, preva; // for slam guessi
	float dx, dy, da;
	bool quit;
	uint32_t delaytime;
	// commandline args
	float reg_pos, reg_ang, grid_scale, init_x, init_y, init_ang;
	int grid_size;
	string waypoint_fname;

	// grab command-line options
	parseCmdLine(argc, argv, &reg_pos, &reg_ang, 
			&grid_size, &grid_scale, &waypoint_fname, 
			&init_x, &init_y, &init_ang);

	setLIDARSpin(&ser, false);
	setLED(LED_GREEN, false);
	setLED(LED_BLUE, false);
	setLED(LED_WHITE, false);

	prevx = init_x;
	prevy = init_y;
	preva = init_ang;

	// STEP 1: initialize classes
	if (DEBUG) cout << "Initializing..." << endl;
	log.init("logfile", true); // log file
	slammer.init(grid_size,grid_size,grid_scale);
	slammer.currx = init_x;
	slammer.curry = init_y;
	slammer.currang = init_ang;
	slammer.setRegularization(reg_pos,reg_pos,reg_ang);
	nav.init(&hex, &slammer, &log, init_x,init_y,init_ang);
	ser.init_old("/dev/ttymxc3", false); // serial comm with Due
#ifdef MANUAL
	if (initSDL(screen, joy) != 0) return -1;
#endif


	// STEP 2: load waypoints for autonav
	if (waypoint_fname == "")
	{
		// dummy target 1m forward
		nav.addTarget(init_x+100.0*cos(init_ang), 
				init_y+100.*sin(init_ang), 10.0);
	} else {
		// load from file
		nav.loadTargets(waypoint_fname);
	}


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
	getRemoteButtonPress(7, true); // blocking wait for Start button, manual.cpp
#else
	// wait for init button
	quit = false;
	lasttime = getTime();
	setLED(LED_WHITE, true);
	while (!quit)
	{
		if (getTime() - lasttime > 0.5) setLED(LED_WHITE, false);
		if (getTime() - lasttime > 1.0)
		{
			lasttime = getTime();
			setLED(LED_WHITE, true);
		}
		quit = getButtonPress(BUTTON_2, false);
	}
	setLED(LED_WHITE, false);
	usleep(250000);
#endif

	// STEP 4: enable hardware
	enableServos(&ser); // tell servos to turn on
	usleep(50*1000); // don't send stuff too quickly..
	setLIDARSpin(&ser, true); // lidar motor spin up
	// stand up safely while LIDAR gets up to speed
	if (DEBUG) cout << "Performing Safe Stand.." << endl;
	performSafeStand(&hex, &ser);
	setLED(LED_WHITE, true);
	usleep(1000*1000*3); // wait more for LIDAR

	// wait for user input before using lidar scans
#ifdef MANUAL
	if (DEBUG) cout << "Press Start to continue." << endl;
	getRemoteButtonPress(7, true); // blocking wait for Start button, manual.cpp
#else
	// wait for init button
	quit = false;
	lasttime = getTime();
	setLED(LED_BLUE, true);
	while (!quit)
	{
		if (getTime() - lasttime > 0.5) setLED(LED_BLUE, false);
		if (getTime() - lasttime > 1.0)
		{
			lasttime = getTime();
			setLED(LED_BLUE, true);
		}
		quit = getButtonPress(BUTTON_3, false);
	}
	setLED(LED_BLUE, false);
	usleep(250000);
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
			setLED(LED_BLUE, true);
			slammer.integrate(lidar_scan, 0.0, 0.0, 0.0);
			delete lidar_scan;
			setLED(LED_BLUE, false);
			scans ++;
		}
#ifdef MANUAL
		quit = getRemoteButtonPress(7, false);
#else
		// check status of button
		quit = getButtonPress(BUTTON_3, false);
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
	setLED(LED_BLUE, true);

	// wait for go button!
#ifndef MANUAL
	quit = false;
	lasttime = getTime();
	setLED(LED_GREEN, true);
	while (!quit)
	{
		if (getTime() - lasttime > 0.5) setLED(LED_GREEN, false);
		if (getTime() - lasttime > 1.0)
		{
			lasttime = getTime();
			setLED(LED_GREEN, true);
		}
		quit = getButtonPress(BUTTON_4, false);
	}
	setLED(LED_GREEN, true);
	usleep(250000);
#endif
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
		quit = getButtonPress(BUTTON_4, false);
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
	// turn off the lights
	setLED(LED_GREEN, false);
	setLED(LED_BLUE, false);
	setLED(LED_WHITE, false);

}

// use tclap library to parse command-line options
void parseCmdLine(int argc, char *argv[], 
		float *reg_pos, float *reg_ang, int *grid_size, float *grid_scale, 
		string *waypoint_fname, float *init_x, float *init_y, float *init_ang)
{
	bool crash;

	try
	{
		TCLAP::CmdLine cmd("Hexapod Code", ' ', "0.9");
		
		// SLAM regularization
		TCLAP::ValueArg<float> arg_reg_pos("p", "reg_pos", 
				"SLAM Regularization for Position", false, 0.3, "float value");
		cmd.add(arg_reg_pos);
		TCLAP::ValueArg<float> arg_reg_ang("a", "reg_ang", 
				"SLAM Regularization for Angle", false, 0.3, "float value");
		cmd.add(arg_reg_ang);

		// autonav waypoint file
		TCLAP::ValueArg<string> arg_waypoints("w", "waypoints", 
				"Autonav Waypoint file", false, "", "filename");
		cmd.add(arg_waypoints);

		// SLAM grid resolution
		TCLAP::ValueArg<int> arg_grid_size("r", "grid_size", 
				"SLAM Grid Size, in pixels", false, 128, "integer value");
		cmd.add(arg_grid_size);

		// SLAM grid scale
		TCLAP::ValueArg<float> arg_grid_scale("s", "grid_scale", 
				"SLAM Grid Scale, cm per pixel", false, 10.0, "float value");
		cmd.add(arg_grid_scale);

		// initial position and angle
		TCLAP::ValueArg<float> arg_init_x("x", "init_x", 
				"Initial x position, in cm", false, 0.0, "float value");
		cmd.add(arg_init_x);
		TCLAP::ValueArg<float> arg_init_y("y", "init_y", 
				"Initial y position, in cm", false, 0.0, "float value");
		cmd.add(arg_init_y);
		TCLAP::ValueArg<float> arg_init_ang("t", "init_ang", 
				"Initial angle, in radians", false, 0.0, "float value");
		cmd.add(arg_init_ang);

		//  have options loaded, parse what we have
		cmd.parse(argc, argv);

		// grab results
		*reg_pos = arg_reg_pos.getValue();
		*reg_ang = arg_reg_ang.getValue();
		*grid_size = arg_grid_size.getValue();
		*grid_scale = arg_grid_scale.getValue();
		*waypoint_fname = arg_waypoints.getValue();
		*init_x = arg_init_x.getValue();
		*init_y = arg_init_y.getValue();
		*init_ang = arg_init_ang.getValue();

		// check values
		crash = false;
		if (*reg_pos < 1e-5 || *reg_pos > 1e7)
			{cout << "ERROR: Invalid SLAM regularization on position" << endl;crash = true;}
		if (*reg_ang < 1e-5 || *reg_ang > 1e7)
			{cout << "ERROR: Invalid SLAM regularization on angle" << endl;crash = true;}
		if (*grid_size < 1 || *grid_size > 4096)
			{cout << "ERROR: Invalid SLAM grid size" << endl;crash = true;}
		if (*grid_scale < 1e-2 || *grid_scale > 1e4)
			{cout << "ERROR: Invalid SLAM grid scale" << endl;crash = true;}
		if (*init_x < -1e4 || *init_x > 1e4)
			{cout << "ERROR: Invalid initial x-pos" << endl;crash = true;}
		if (*init_y < -1e4 || *init_y > 1e4)
			{cout << "ERROR: Invalid initial y-pos" << endl;crash = true;}
		if (*init_ang < -2*PI || *init_ang > 2*PI)
			{cout << "ERROR: Invalid initial angle" << endl;crash = true;}
		if (crash) exit(-1);

	} catch (TCLAP::ArgException &e) {
		cerr << "error: " << e.error() << " for arg " << e.argId() << endl;
	}
}

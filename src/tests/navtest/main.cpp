#include <iostream>
#include <string>
#include <stdio.h>
#include <thread>
#include <cmath>
#include <iomanip>
#include "/home/bgreer/PROJECTS/HEX/LIB_AUTONAV/autonav.h"
#include "/home/bgreer/PROJECTS/HEX/LIB_SLAM/slam.h"
#include "/home/bgreer/PROJECTS/HEX/LIB_HEXAPOD/hexapod.h"

using namespace std;


int main(void)
{
	int ii;
	hexapod hex;
	slam slammer;
	autonav nav;
	double time, lasttime, dt, lastdata, inittime;
	bool done;

	inittime = getTime();
	
	slammer.init(128,128,5.0);

	nav.init(&hex, &slammer, 0,0,0);
	nav.addTarget(100.0, 0.0, 10.0);
	nav.addTarget(100.0, 100.0, 10.0);
	nav.addTarget(0.0,50.0,10.0);
	nav.addTarget(-100.0,100.0,10.0);
	nav.addTarget(-85.0, 30,5.0);
	nav.addTarget(-105.0, -30.0, 5.0);
	nav.addTarget(-100.0,-50.0,10.0);
	nav.addTarget(-10.,0.0,10.0);
	dt = 0.05;


	time = 0.0;
	lasttime = getTime();
	
	done = false;
	while (!done)
	{
		hex.step(dt);

		cout << hex.dr_xpos << "\t" << hex.dr_ypos << "\t" << hex.turning << endl;
		// autonav will set the hexapod speed and turning values
		nav.solve(hex.dr_xpos, hex.dr_ypos, hex.dr_ang);


		// check for end of target list
		nav.anlock.lock();
		done = (nav.currtarget == nav.target_x.size());
		nav.anlock.unlock();
		usleep(10*1000);
	}

	nav.close();
	slammer.close();
}

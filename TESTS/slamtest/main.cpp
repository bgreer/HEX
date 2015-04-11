#include <iostream>
#include <string>
#include <stdio.h>
#include <thread>
#include <cmath>
#include <iomanip>
#include <random>
#include "/home/bgreer/PROJECTS/HEX/LIB_SLAM/slam.h"

using namespace std;

#define NUM_INTEGRATE 10

// make a fake scan for testing
// adds noise, too!
scan* newScan (float realx, float realy, float realang, default_random_engine *gen)
{
	int ii, num;
	float d1, d2, d3, d4, tht, r;
	normal_distribution<float> dist(0.0,1.0);
	scan *ret;

	num = 360;
	ret = new scan(num);

	for (ii=0; ii<num; ii++)
	{
		tht = 2.*PI*ii/((float)num);
		ret->angle[ii] = tht;
		tht += realang;
		ret->weight[ii] = 1.0;
		d1 = min((100.-realy)/sin(tht), 1000.);
		if (d1 < 0.0) d1 = 1000.;
		d2 = min((-100.-realy)/sin(tht), 1000.);
		if (d2 < 0.0) d2 = 1000.;
		d3 = min((200.-realx)/cos(tht), 1000.);
		if (d3 < 0.0) d3 = 1000.;
		d4 = min((-200.-realx)/cos(tht), 1000.);
		if (d4 < 0.0) d4 = 1000.;
		r = min(min(d1, d2), min(d3, d4));
		r *= (1.0 + dist(*gen)*0.01);
		ret->dist[ii] = r;
	}

	return ret;
}

int main (void)
{
	int ii;
	default_random_engine gen;
	slam slammer(64,64,10.0);
	scan *s;
	float xg, yg, ag;

	slammer.setRegularization(0.0,0.0,0.0);


	// make some fake scans
	for (ii=0; ii<NUM_INTEGRATE; ii++)
	{
		s = newScan(0.0, 0.0, 0.0, &gen);
		slammer.integrate(s, 0.0, 0.0, 0.0);
		delete s;
	}

	for (ii=0; ii<25; ii++)
	{
		// generate scan to match
		s = newScan(ii*2, ii*1, 0.5, &gen);
		// guess values, from kinematics?
		xg = 0.0;
		yg = 0.0;
		ag = 0.5;

		slammer.step(s, xg, yg, ag);
		cout << slammer.currx << " " << slammer.curry << " " << slammer.currang << " " << ii*2 << endl;
//		slammer.integrate(s, xg, yg, ag);

//		slammer.outputMap("map");

		delete s;
	}
	slammer.outputMap("map");
	return EXIT_SUCCESS;
}

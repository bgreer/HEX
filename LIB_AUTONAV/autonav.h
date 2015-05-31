#ifndef AUTONAV_H
#define AUTONAV_H
#include "/home/bgreer/PROJECTS/HEX/LIB_HEXAPOD/hexapod.h"
#include "/home/bgreer/PROJECTS/HEX/LIB_SLAM/slam.h"
#include <sys/time.h>
#include <vector>
#include <mutex>
#include <thread>

// minimum time needed to spend near target before moving to next one
// in seconds
#define AUTONAV_TARGET_MINTIME 0.001

// how much to weight the map, usually > 100
#define MAP_WEIGHT 100.0

// in cm
#define AN_MIN_TARGET_DIST 20.0

class an_node
{
public:
	float f_score = 0;
	float g_score = 0;
	an_node *pathtracer;
	uint16_t xpos, ypos;

	an_node (uint16_t x, uint16_t y, float f_score = 0.0, float g_score = 0.0)
	{
		xpos = x;
		ypos = y;
	}
};

class autonav
{
public:
	// targets:
	int currtarget;
	double currtarget_time; // how much time we've spent at the current target
	vector<float> target_x, target_y, target_r;
	double lastsolve;

	// for async solving
	thread listener;
	bool running, computing, triggered;
	float cx, cy, ca; // current position
	hexapod *hex;
	slam *slammer;
	mutex anlock;

	// constructor
	autonav ();
	void init(hexapod *hex0, slam *slammer0, float x, float y, float a);
	void addTarget(float xpos, float ypos, float rad);
	void solve(float currx, float curry, float currang);
	void close();
};

double getTime();
void autonav_loop (autonav *an);
#endif

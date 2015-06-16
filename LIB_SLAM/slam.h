#ifndef SLAM_H
#define SLAM_H
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <unistd.h>
#include <cmath>
#include <complex>
#include <thread>
#include <mutex>
#include <string.h>
#include "fftw3.h"
#include "scan.h"


using namespace std;

#ifndef PI
#define PI 3.14159265359
#endif
#define SLAM_MAXITER 1000
#define STEPTOL 1e-5
#define SLAM_MINDIST 10.0

#define SLAM_THREADS

class slam
{
public:
	int nx, ny; // size of map in pixels
	float scale; // cm per pixel
	float maxdist;
	float *map, *map_filt, *map_dx, *map_dy; // 2d, let y be fastest dimension
	mutex slam_mutex;
	thread slam_thread;
	scan *currscan;
	float guessx, guessy, guessang;
	bool running, computing;
	float currx, curry, currang; // current position
	float reg_a, reg_b, reg_c; // regularization params

	// filters
	complex<double> *filt, *filt_dx, *filt_dy;
	fftw_plan filter_plan1, filter_plan2;
	complex<double> *filter_ftmap, *filter_input, *filter_output;
	// filt_dy is

	slam ();
	void submitScan (scan *s, float x_guess, float y_guess, float ang_guess);
	void init (int v, int y, float s);
	void setRegularization (float valx, float valy, float vala);
	void integrate (scan *s, float x_val, float y_val, float ang_val);
	void filter ();
	bool step (scan *s, float x_guess, float y_guess, float ang_guess);
	void outputMap (const char *fname);
	void close ();

	~slam ()
	{
		fftw_destroy_plan(filter_plan1);
		fftw_destroy_plan(filter_plan2);
		delete [] filter_input;
		delete [] filter_output;
		delete [] filter_ftmap;
		delete [] filt;
		delete [] filt_dx;
		delete [] filt_dy;
		delete [] map;
		delete [] map_filt;
		delete [] map_dx;
		delete [] map_dy;
		fftw_cleanup();
	}
};

void slam_loop (slam *slammer);
bool withinBounds (int x, int y, int x0, int x1, int y0, int y1);
#endif

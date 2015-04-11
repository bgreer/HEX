#include <iostream>
#include <fstream>
#include <stdio.h>
#include <cmath>
#include <complex>
#include <mutex>
#include <string.h>
#include "fftw3.h"
#include "scan.h"

using namespace std;

#define PI 3.14159265359
#define MAXITER 1000
#define STEPTOL 1e-4

class slam
{
public:
	int nx, ny; // size of map in pixels
	float scale; // cm per pixel
	float maxdist;
	float *map, *map_filt, *map_dx, *map_dy; // 2d, let y be fastest dimension
	mutex maplock; // for other threads accessing the map
	float currx, curry, currang; // current position
	float reg_a, reg_b, reg_c; // regularization params

	// filters
	complex<double> *filt, *filt_dx, *filt_dy;
	// filt_dy is

	slam (int x, int y, float s);
	void setRegularization (float valx, float valy, float vala);
	void integrate (scan *s, float x_val, float y_val, float ang_val);
	void filter ();
	bool step (scan *s, float x_guess, float y_guess, float ang_guess);
	void outputMap (char *fname);

	~slam ()
	{
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

bool withinBounds (int x, int y, int x0, int x1, int y0, int y1);

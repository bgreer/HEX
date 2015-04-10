#include <iostream>
#include <fstream>
#include <stdio.h>
#include <cmath>
#include <complex>
#include "fftw3.h"
#include "scan.h"
using namespace std;
#define PI 3.14159265359
class slam
{
public:
	int nx, ny; // size of map in pixels
	float scale; // cm per pixel
	float maxdist;
	float *map, *map_filt, *map_dx, *map_dy; // 2d, let y be fastest dimension
	float currx, curry, currang; // current position

	// filters
	complex<double> *filt, *filt_dx, *filt_dy;
	// filt_dy is

	slam (int x, int y, float s);
	void integrate (scan *s, float x_val, float y_val, float ang_val);
	void filter ();
	void step (scan *s, float x_guess, float y_guess, float ang_guess);
	void outputMap (char *fname);

	~slam ()
	{
		free(map);
		free(map_filt);
		free(map_dx);
		free(map_dy);
	}
};

bool withinBounds (int x, int y, int x0, int x1, int y0, int y1);

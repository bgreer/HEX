#include "fftw3.h"

class slam
{

	int nx, ny; // size of map in pixels
	float scale; // cm per pixel
	float *map, *map_filt, *map_dx, *map_dy;
	float currx, curry, currang; // current position

	// filters
	fftw_complex *filt, *filt_dx, *filt_dy;
	// filt_dy is

	// init with size in pixels and scale (for comparing to scan)
	slam (int x, int y, float s)
	{
		nx = x;
		ny = y;
		scale = s;

		// precompute fourier filters
		filt = (fftw_complex*) malloc(nx*ny*sizeof(fftw_complex));

		// make space for primary map
		map = (float*) malloc(nx*ny*sizeof(float));
		// make space for filtered maps
		map_filt = (float*) malloc(nx*ny*sizeof(float));
		map_dx = (float*) malloc(nx*ny*sizeof(float));
		map_dy = (float*) malloc(nx*ny*sizeof(float));
	}

	// given a scan and known position, add data to map
	// no slam, just mapping
	// useful for robot getting bearings before slam stepping
	integrate (scan *s, float x_val, float y_val, float ang_val)
	{

	}

	// given a scan and estimate of position, perform SLAM step
	step (scan *s, float x_guess, float y_guess, float ang_guess)
	{
		// filter map in fourier space
		// set up data for iteration
		// iterate until converged or failed
		// compute psi and jacobian
		// take step
		// compute new psi
		// redo step size (if necessary)
		// if converged, apply new position and heading
	}

	~slam ()
	{
		free(map);
		free(map_filt);
		free(map_dx);
		free(map_dy);
	}
};

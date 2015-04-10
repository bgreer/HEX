
#include "slam.h"


// init with size in pixels and scale (for comparing to scan)
slam::slam (int x, int y, float s)
{
	int ix, iy;
	float *kernel, r;
	complex<double> *input, *output;
	double kx, ky;
	complex<double> val_i(0,1);
	fftw_plan plan;

	nx = x;
	ny = y;
	scale = s;
	maxdist = sqrt(nx*nx*0.25 + ny*ny*0.25)*scale;
	
	input = new complex<double> [nx*ny];
	output = new complex<double> [nx*ny];
	plan = fftw_plan_dft_2d(nx, ny, 
			reinterpret_cast<fftw_complex*>(input), 
			reinterpret_cast<fftw_complex*>(output), 
			FFTW_FORWARD, FFTW_ESTIMATE);
	// precompute distance kernel
	for (ix=0; ix<nx; ix++)
	{
		for (iy=0; iy<ny; iy++)
		{
			r = sqrt(pow(ix-nx/2,2) + pow(iy-ny/2,2));
			input[ix*ny+iy] = exp(-pow(r/10.,2));
		}
	}
	fftw_execute(plan);
	filt = new complex<double> [nx*ny]; // distance kernel
	filt_dx = new complex<double> [nx*ny]; // kernel*i*kx
	filt_dy = new complex<double> [nx*ny]; // kernel*i*ky
	for (ix=0; ix<nx; ix++)
	{
		for (iy=0; iy<ny; iy++)
		{
			filt[ix*ny+iy] = output[ix*ny+iy];
			kx = ix;
			if (ix >= nx/2) kx = ix-nx;
			ky = iy;
			if (iy >= ny/2) ky = iy-ny;
			filt_dx[ix*ny+iy] = output[ix*ny+iy]*val_i*kx;
			filt_dy[ix*ny+iy] = output[ix*ny+iy]*val_i*ky;
		}
	}
	delete [] input;
	delete [] output;
	fftw_destroy_plan(plan);
	
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
void slam::integrate (scan *s, float x_val, float y_val, float ang_val)
{
	int ii, xpos, ypos;

	// error checking
	if (s->num == 0)
	{
		cout << "WARNING: scan of size 0 passed to integrator!" << endl;
		return;
	}

	for (ii=0; ii<s->num; ii++)
	{
		xpos = (int)((s->dist[ii]*cos(s->angle[ii]+ang_val)+x_val)/scale + nx/2);
		ypos = (int)((s->dist[ii]*sin(s->angle[ii]+ang_val)+y_val)/scale + ny/2);
		if (withinBounds(xpos,ypos,0,nx-1,0,ny-1))
			map[xpos*ny+ypos] = min(map[xpos*ny+ypos]+s->dist[ii]*0.3/maxdist, 1.0);
	}
}

void slam::filter ()
{
	int ix, iy;
	complex<double> *ftmap, *input, *output;
	double n2;
	fftw_plan plan1, plan2;

	n2 = (double)nx*ny;
	
	// filter map in fourier space
	input = new complex<double> [nx*ny];
	output = new complex<double> [nx*ny];
	ftmap = new complex<double> [nx*ny];
	plan1 = fftw_plan_dft_2d(nx, ny, 
			reinterpret_cast<fftw_complex*>(input), 
			reinterpret_cast<fftw_complex*>(ftmap), 
			FFTW_FORWARD, FFTW_ESTIMATE);
	plan2 = fftw_plan_dft_2d(nx, ny, 
			reinterpret_cast<fftw_complex*>(input), 
			reinterpret_cast<fftw_complex*>(output), 
			FFTW_BACKWARD, FFTW_ESTIMATE);

	// load map into input
	for (ix=0; ix<nx; ix++)
		for (iy=0; iy<ny; iy++)
			input[ix*ny+iy] = map[ix*ny+iy];
	fftw_execute(plan1);
	// apply filter
	for (ix=0; ix<nx; ix++)
		for (iy=0; iy<ny; iy++)
			input[ix*ny+iy] = ftmap[ix*ny+iy] * filt[ix*ny+iy];
	fftw_execute(plan2);
	// unload into map_filt, shifted
	for (ix=0; ix<nx; ix++)
		for (iy=0; iy<ny; iy++)
			map_filt[ix*ny+iy] = real(output[((ix+nx/2) % nx)*ny+((iy+ny/2) % ny)])/n2;
	// apply filt-dx
	for (ix=0; ix<nx; ix++)
		for (iy=0; iy<ny; iy++)
			input[ix*ny+iy] = ftmap[ix*ny+iy] * filt_dx[ix*ny+iy];
	fftw_execute(plan2);
	// unload into map_dx, shifted
	for (ix=0; ix<nx; ix++)
		for (iy=0; iy<ny; iy++)
			map_dx[ix*ny+iy] = real(output[((ix+nx/2) % nx)*ny+((iy+ny/2) % ny)])/n2;
	// apply fil-dy
	for (ix=0; ix<nx; ix++)
		for (iy=0; iy<ny; iy++)
			input[ix*ny+iy] = ftmap[ix*ny+iy] * filt_dy[ix*ny+iy];
	fftw_execute(plan2);
	// unload into map_dy, shifted
	for (ix=0; ix<nx; ix++)
		for (iy=0; iy<ny; iy++)
			map_dy[ix*ny+iy] = real(output[((ix+nx/2) % nx)*ny+((iy+ny/2) % ny)])/n2;
	fftw_destroy_plan(plan1);
	fftw_destroy_plan(plan2);
	delete [] input;
	delete [] output;
	delete [] ftmap;
}

// given a scan and estimate of position, perform SLAM step
void slam::step (scan *s, float x_guess, float y_guess, float ang_guess)
{
	// set up data for iteration
	// iterate until converged or failed
	// compute psi and jacobian
	// take step
	// compute new psi
	// redo step size (if necessary)
	// if converged, apply new position and heading
}

void slam::outputMap (char *fname)
{
	int ii, ij, ix, iy;
	ofstream file;

	filter();

	file.open(fname, ios::out);
	for (ii=0; ii<nx; ii++)
	{
		for (ij=0; ij<ny; ij++)
		{
			file << ii << " " << ij << " " << map[ii*ny+ij] << " " << 
				map_filt[ii*ny+ij] << " " << map_dx[ii*ny+ij] << " " << 
				map_dy[ii*ny+ij] << endl;
		}
		file << endl;
	}
	file.close();
}

// inclusive
bool withinBounds (int x, int y, int x0, int x1, int y0, int y1)
{
	if (x < x0 || x > x1) return false;
	if (y < y0 || y > y1) return false;
	return true;
}

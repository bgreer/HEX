
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
	// TODO: allow modification of regularization values
	reg_a = 1.0;
	reg_b = 1.0;
	reg_c = 1.0;
	
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
			input[ix*ny+iy] = exp(-pow(r*scale/100.,2));
			input[ix*ny+iy] = 1./(r + 3.0);
//			if (r > nx*0.1/2) input[ix*ny+iy] = 0;
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
			map[xpos*ny+ypos] = min(map[xpos*ny+ypos]+s->dist[ii]*0.5/maxdist, 1.0);
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
			map_filt[ix*ny+iy] = -real(output[((ix+nx/2) % nx)*ny+((iy+ny/2) % ny)])/n2;
	// apply filt-dx
	for (ix=0; ix<nx; ix++)
		for (iy=0; iy<ny; iy++)
			input[ix*ny+iy] = ftmap[ix*ny+iy] * filt_dx[ix*ny+iy];
	fftw_execute(plan2);
	// unload into map_dx, shifted
	for (ix=0; ix<nx; ix++)
		for (iy=0; iy<ny; iy++)
			map_dx[ix*ny+iy] = -real(output[((ix+nx/2) % nx)*ny+((iy+ny/2) % ny)])/n2;
	// apply fil-dy
	for (ix=0; ix<nx; ix++)
		for (iy=0; iy<ny; iy++)
			input[ix*ny+iy] = ftmap[ix*ny+iy] * filt_dy[ix*ny+iy];
	fftw_execute(plan2);
	// unload into map_dy, shifted
	for (ix=0; ix<nx; ix++)
		for (iy=0; iy<ny; iy++)
			map_dy[ix*ny+iy] = -real(output[((ix+nx/2) % nx)*ny+((iy+ny/2) % ny)])/n2;
	fftw_destroy_plan(plan1);
	fftw_destroy_plan(plan2);
	delete [] input;
	delete [] output;
	delete [] ftmap;
}

// given a scan and estimate of position, perform SLAM step
// return true if found answer
bool slam::step (scan *s, float x_guess, float y_guess, float ang_guess)
{
	int iter, xind, yind, ii;
	float step, psi, psi_trial;
	float x_trial, y_trial, a_trial, count, inv;
	float xval, yval, aval;
	float J[3]; // jacobian
	float M[3]; // step

	// error checking
	if (s->num == 0)
	{
		cout << "WARNING: cannot do SLAM step with a scan of size 0!" << endl;
		return false;
	}

	// update filtered map
	filter();
	// set up data for iteration
	xval = x_guess;
	yval = y_guess;
	aval = ang_guess;
	step = 1.0;
	iter = 0;
	// iterate until converged or failed
	while (iter < MAXITER && step > STEPTOL)
	{
		// compute psi and jacobian
		psi = 0.0;
		count = 0.0;
		J[0] = 0.0; J[1] = 0.0; J[2] = 0.0;
		for (ii=0; ii<s->num; ii++)
		{
			xind = (s->dist[ii]*cos(s->angle[ii]+aval)+xval)/scale + nx/2;
			yind = (s->dist[ii]*sin(s->angle[ii]+aval)+yval)/scale + ny/2;
			if (withinBounds(xind,yind,0,nx-1,0,ny-1))
			{
				psi += map_filt[xind*ny+yind];
				J[0] += map_dx[xind*ny+yind];
				J[1] += map_dy[xind*ny+yind];
				J[2] += -map_dx[xind*ny+yind]*s->dist[ii]*sin(s->angle[ii]+aval)/scale + 
					map_dy[xind*ny+yind]*s->dist[ii]*cos(s->angle[ii]+aval)/scale;
				count += 1.0;
			}
		}
		// normalize
		psi /= count;
		J[0] /= count; J[1] /= count; J[2] /= count;
		// compute step
		inv = 1.0/(J[0]*J[0] + J[1]*J[1] + J[2]*J[2]);
		M[0] = J[0]*inv;
		M[1] = J[1]*inv;
		M[2] = J[2]*inv;
		// take step
		cout << "step " << step*psi*M[0] << " " << step*psi*M[1] << " " << step*psi*M[2] << endl;
		x_trial = xval + 1.*step*psi*M[0];
		y_trial = yval + 1.*step*psi*M[1];
		a_trial = aval + 0.01*step*psi*M[2];
		// compute new psi based on trial solution
		psi_trial = 0.0;
		count = 0.0;
		for (ii=0; ii<s->num; ii++)
		{
			xind = (s->dist[ii]*cos(s->angle[ii]+a_trial)+x_trial)/scale + nx/2;
			yind = (s->dist[ii]*sin(s->angle[ii]+a_trial)+y_trial)/scale + ny/2;
			if (withinBounds(xind,yind,0,nx-1,0,ny-1))
			{
				psi_trial += map_filt[xind*ny+yind];
				count += 1.0;
			}
		}
		psi_trial /= count;
		cout << "psi " << psi << "  " << psi_trial << endl;
		// redo step size (if necessary)
		if (psi_trial <= psi)
		{
			// apply step
			xval = x_trial;
			yval = y_trial;
			aval = a_trial;
			iter ++;
		} else {
			step *= 0.5; // take a smaller step
		}
		cout << iter << " " << xval << " " << yval << " " << aval << " " << step << endl;
		cout << endl;
	}
	// if converged, apply new position and heading
	if (iter == MAXITER)
	{
		return false;
	} else {
		// store result internally
		currx = xval;
		curry = yval;
		currang = aval;
		// add scan to map
		cout << "SUCCESS" << endl;
		for (ii=0; ii<100; ii++)
			integrate(s, xval, yval, aval);
		return true;
	}
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

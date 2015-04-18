
#include "slam.h"


// init with size in pixels and scale (for comparing to scan)
slam::slam (int x, int y, float s)
{
	int ix, iy;
	float *kernel, r;
	complex<double> *input, *output;
	double kx, ky, width;
	complex<double> val_i(0,1);
	fftw_plan plan;

	maplock.lock();

	nx = x;
	ny = y;
	scale = s;
	maxdist = sqrt(nx*nx*0.25 + ny*ny*0.25)*scale;
	reg_a = 0.1;
	reg_b = 0.1;
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
			width = 50./scale;
			input[ix*ny+iy] = width/(pow(r,1.5) + width);
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
	map = new float [nx*ny];
	memset(map, 0x00, nx*ny*sizeof(float));
	// make space for filtered maps
	map_filt = new float [nx*ny];
	memset(map_filt, 0x00, nx*ny*sizeof(float));
	map_dx = new float [nx*ny];
	memset(map_dx, 0x00, nx*ny*sizeof(float));
	map_dy = new float [nx*ny];
	memset(map_dy, 0x00, nx*ny*sizeof(float));
	maplock.unlock();
}

void slam::setRegularization (float valx, float valy, float vala)
{
	reg_a = valx;
	reg_b = valy;
	reg_c = vala;
}

// given a scan and known position, add data to map
// no slam, just mapping
// useful for robot getting bearings before slam stepping
void slam::integrate (scan *s, float x_val, float y_val, float ang_val)
{
	int ii, xpos, ypos, ind;
	float tht, diff, dist, ang;

	// error checking
	if (s->num == 0)
	{
		cout << "WARNING: scan of size 0 passed to integrator!" << endl;
		return;
	}


	maplock.lock();
	// decay map???
	for (xpos=0; xpos<nx; xpos++)
	{
		for (ypos=0; ypos<ny; ypos++)
		{
			tht = atan2(ypos-ny/2-y_val/scale, xpos-nx/2-x_val/scale);
			dist = sqrt(pow((ypos-ny/2)*scale-y_val,2)+pow((xpos-nx/2)*scale-x_val,2));
			diff = 1e6;
			for (ii=0; ii<s->num; ii++)
			{
				ang = fmod(s->angle[ii],(float)(2.*PI));
				if (ang > PI) ang -= 2.*PI;

				if (fabs(tht-ang) < diff)
				{
					diff = fabs(tht-ang);
					ind = ii;
				}
			}
			if (dist < s->dist[ind])
				map[xpos*ny+ypos] *= 0.95;
			//else map[xpos*ny+ypos] *= 0.9999;
		}
	}


	for (ii=0; ii<s->num; ii++)
	{
		xpos = (int)((s->dist[ii]*cos(s->angle[ii]+ang_val)+x_val)/scale + nx/2);
		ypos = (int)((s->dist[ii]*sin(s->angle[ii]+ang_val)+y_val)/scale + ny/2);
		if (withinBounds(xpos,ypos,0,nx-1,0,ny-1))
			map[xpos*ny+ypos] = min(map[xpos*ny+ypos]+s->dist[ii]*0.5/maxdist, 1.0);
	}
	maplock.unlock();
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

	maplock.lock();
	// load map into input
	for (ix=0; ix<nx; ix++)
		for (iy=0; iy<ny; iy++)
			input[ix*ny+iy] = map[ix*ny+iy];
	maplock.unlock();
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
		psi = reg_a*pow(xval-x_guess,2) + reg_b*pow(yval-y_guess,2) 
			+ reg_c*pow(aval-ang_guess,2);
		J[0] = 0.0; J[1] = 0.0; J[2] = 0.0;
		for (ii=0; ii<s->num; ii++)
		{
			xind = (s->dist[ii]*cos(s->angle[ii]+aval)+xval)/scale + nx/2;
			yind = (s->dist[ii]*sin(s->angle[ii]+aval)+yval)/scale + ny/2;
			if (withinBounds(xind,yind,0,nx-1,0,ny-1))
			{
				psi += map_filt[xind*ny+yind];
				J[0] += map_dx[xind*ny+yind] + 2.*reg_a*(xval-x_guess);
				J[1] += map_dy[xind*ny+yind] + 2.*reg_b*(yval-y_guess);
				J[2] += -map_dx[xind*ny+yind]*s->dist[ii]*sin(s->angle[ii]+aval)/scale + 
					map_dy[xind*ny+yind]*s->dist[ii]*cos(s->angle[ii]+aval)/scale
					+ 2.*reg_c*(aval-ang_guess);
			} else {
				psi += 0.0; // far away
			}
		}
		// compute step
		inv = 1.0/(J[0]*J[0] + J[1]*J[1] + J[2]*J[2]);
		M[0] = J[0]*inv;
		M[1] = J[1]*inv;
		M[2] = J[2]*inv;
		// take step
//		cout << "step " << step*psi*M[0] << " " << step*psi*M[1] << " " << step*psi*M[2] << endl;
		x_trial = xval + 50.*step*psi*M[0];
		y_trial = yval + 50.*step*psi*M[1];
		a_trial = aval + 0.1*step*psi*M[2];
		// compute new psi based on trial solution
		psi_trial = reg_a*pow(x_trial-x_guess,2) + reg_b*pow(y_trial-y_guess,2) 
			+ reg_c*pow(a_trial-ang_guess,2);
		for (ii=0; ii<s->num; ii++)
		{
			xind = (s->dist[ii]*cos(s->angle[ii]+a_trial)+x_trial)/scale + nx/2;
			yind = (s->dist[ii]*sin(s->angle[ii]+a_trial)+y_trial)/scale + ny/2;
			if (withinBounds(xind,yind,0,nx-1,0,ny-1))
				psi_trial += map_filt[xind*ny+yind];
		}
//		cout << "psi " << psi << "  " << psi_trial << endl;
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
//		cout << iter << " " << xval << " " << yval << " " << aval << " " << step << endl;
//		cout << endl;
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
//		cout << "SUCCESS" << endl;
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

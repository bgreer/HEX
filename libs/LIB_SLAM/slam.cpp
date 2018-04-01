
#include "slam.h"

slam::slam ()
{
}

void slam::submitScan (scan *s, float x_guess, float y_guess, float ang_guess)
{
	slam_mutex.lock();
	if (currscan != NULL)
		delete currscan;
	currscan = s->copy();
	guessx = x_guess;
	guessy = y_guess;
	guessang = ang_guess;
	slam_mutex.unlock();
}

// wait for new scans, compute things
void slam_loop (slam *slammer)
{
	float xg, yg, ag;
	bool newscan;
	scan *s;

	newscan = false;

	while (slammer->running)
	{
		// check for a scan to run
		slammer->slam_mutex.lock();
		if (slammer->currscan != NULL)
		{
			// grab info
			xg = slammer->guessx;
			yg = slammer->guessy;
			ag = slammer->guessang;
			s = slammer->currscan->copy();
			delete slammer->currscan;
			slammer->currscan = NULL;
			slammer->computing = true;
			newscan = true;
		}
		slammer->slam_mutex.unlock();

		// we have a local copy of a new scan, run it
		if (newscan)
		{
			cout << "stepping slam.." << endl;
			slammer->step(s, xg, yg, ag);
			cout << "done stepping" << endl;
			newscan = false;
			slammer->slam_mutex.lock();
			slammer->computing = false;
			slammer->slam_mutex.unlock();
		}
		
		usleep(1000);
	}
}

void slam::close ()
{
	while (computing) usleep(1000);
	running = false;
	slam_thread.join();
}

// init with size in pixels and scale (for comparing to scan)
void slam::init (int x, int y, float s)
{
	int ix, iy;
	float *kernel, r;
	complex<double> *input, *output;
	double kx, ky, width;
	complex<double> val_i(0,1);
	fftw_plan plan;

#ifdef SLAM_THREADS
	fftw_init_threads();
#endif

	slam_mutex.lock();
	currscan = NULL;
	nx = x;
	ny = y;
	scale = s;
	currx = 0;
	curry = 0;
	currang = 0;
	maxdist = sqrt(nx*nx*0.25 + ny*ny*0.25)*scale;
	reg_a = 0.1;
	reg_b = 0.1;
	reg_c = 1.0;
	
	input = new complex<double> [nx*ny];
	output = new complex<double> [nx*ny];
#ifdef SLAM_THREADS
	fftw_plan_with_nthreads(3);
#endif
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
			input[ix*ny+iy] = width/(pow(r,2.0) + width);
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
	filter_input = new complex<double> [nx*ny];
	filter_output = new complex<double> [nx*ny];
	filter_ftmap = new complex<double> [nx*ny];
	filter_plan1 = fftw_plan_dft_2d(nx, ny, 
			reinterpret_cast<fftw_complex*>(filter_input), 
			reinterpret_cast<fftw_complex*>(filter_ftmap), 
			FFTW_FORWARD, FFTW_ESTIMATE);
	filter_plan2 = fftw_plan_dft_2d(nx, ny, 
			reinterpret_cast<fftw_complex*>(filter_input), 
			reinterpret_cast<fftw_complex*>(filter_output), 
			FFTW_BACKWARD, FFTW_ESTIMATE);


	computing = false;
	running = true;
	slam_mutex.unlock();
	slam_thread = thread(slam_loop, this);

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
	int ii, xpos, ypos, ind, x0, x1, y0, y1;
	int cx, cy;
	float tht, diff, dist, ang;

	// error checking
	if (s->num == 0)
	{
		cout << "WARNING: scan of size 0 passed to integrator!" << endl;
		return;
	}


	slam_mutex.lock();
	for (ii=0; ii<s->num; ii++)
	{
		if (s->dist[ii] > SLAM_MINDIST)
		{
			xpos = (int)((s->dist[ii]*cos(s->angle[ii]+ang_val)+x_val)/scale + nx/2);
			ypos = (int)((s->dist[ii]*sin(s->angle[ii]+ang_val)+y_val)/scale + ny/2);
			if (withinBounds(xpos,ypos,0,nx-1,0,ny-1))
			{
				map[xpos*ny+ypos] = 1.0;
			}
		}
	}

	// decay map???
	cx = currx/scale+nx/2;
	cy = curry/scale+ny/2;
	x0 = (int)max(cx-600./scale,0.);
	x1 = (int)min(cx+600./scale,(double)nx);
	y0 = (int)max(cy-600./scale,0.);
	y1 = (int)min(cy+600./scale,(double)ny);
	for (xpos=x0; xpos<x1; xpos++)
	{
		for (ypos=x0; ypos<y1; ypos++)
		{
			tht = atan2(ypos-ny/2-curry/scale, xpos-nx/2-currx/scale);
			dist = sqrt(pow((ypos-ny/2)*scale-curry,2)+pow((xpos-nx/2)*scale-currx,2));
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
			if (dist < s->dist[ind] && s->dist[ind] > SLAM_MINDIST)
				map[xpos*ny+ypos] *= 0.50 + 0.2*dist/s->dist[ind];
		}
	}


	slam_mutex.unlock();
}

void slam::filter ()
{
	int ix, iy;
	double n2;

	n2 = (double)nx*ny;
	
	// filter map in fourier space

	slam_mutex.lock();
	// load map into input
	for (ix=0; ix<nx; ix++)
		for (iy=0; iy<ny; iy++)
			filter_input[ix*ny+iy] = map[ix*ny+iy];
	slam_mutex.unlock();
	fftw_execute(filter_plan1);
	
	// apply filter
	for (ix=0; ix<nx; ix++)
		for (iy=0; iy<ny; iy++)
			filter_input[ix*ny+iy] = filter_ftmap[ix*ny+iy] * filt[ix*ny+iy];
	fftw_execute(filter_plan2);
	// unload into map_filt, shifted
	slam_mutex.lock();
	for (ix=0; ix<nx; ix++)
		for (iy=0; iy<ny; iy++)
			map_filt[ix*ny+iy] = -real(filter_output[((ix+nx/2) % nx)*ny+((iy+ny/2) % ny)])/n2;
	slam_mutex.unlock();
	
	// apply filt-dx
	for (ix=0; ix<nx; ix++)
		for (iy=0; iy<ny; iy++)
			filter_input[ix*ny+iy] = filter_ftmap[ix*ny+iy] * filt_dx[ix*ny+iy];
	fftw_execute(filter_plan2);
	// unload into map_dx, shifted
	for (ix=0; ix<nx; ix++)
		for (iy=0; iy<ny; iy++)
			map_dx[ix*ny+iy] = -real(filter_output[((ix+nx/2) % nx)*ny+((iy+ny/2) % ny)])/n2;
	
	// apply fil-dy
	for (ix=0; ix<nx; ix++)
		for (iy=0; iy<ny; iy++)
			filter_input[ix*ny+iy] = filter_ftmap[ix*ny+iy] * filt_dy[ix*ny+iy];
	fftw_execute(filter_plan2);
	// unload into map_dy, shifted
	for (ix=0; ix<nx; ix++)
		for (iy=0; iy<ny; iy++)
			map_dy[ix*ny+iy] = -real(filter_output[((ix+nx/2) % nx)*ny+((iy+ny/2) % ny)])/n2;
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
	while (iter < SLAM_MAXITER && step > STEPTOL)
	{
		// compute psi and jacobian
		psi = reg_a*pow(xval-x_guess,2) + reg_b*pow(yval-y_guess,2) 
			+ reg_c*pow(aval-ang_guess,2);
		J[0] = 0.0; J[1] = 0.0; J[2] = 0.0;
		for (ii=0; ii<s->num; ii++)
		{
			if (s->dist[ii] > SLAM_MINDIST)
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
			if (s->dist[ii] > SLAM_MINDIST)
			{
				xind = (s->dist[ii]*cos(s->angle[ii]+a_trial)+x_trial)/scale + nx/2;
				yind = (s->dist[ii]*sin(s->angle[ii]+a_trial)+y_trial)/scale + ny/2;
				if (withinBounds(xind,yind,0,nx-1,0,ny-1))
					psi_trial += map_filt[xind*ny+yind];
			}
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
	if (iter == SLAM_MAXITER)
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

void slam::outputMap (const char *fname)
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

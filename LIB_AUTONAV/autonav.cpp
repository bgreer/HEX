#include "autonav.h"
double getTime()
{
	double ret;
	struct timeval tv;
	gettimeofday(&tv, NULL);
	ret = (tv.tv_sec) + tv.tv_usec*1e-6;
	return ret;
}

void autonav_loop (autonav *an)
{
	float cx, cy, ca; // current position
	float ctx, cty, ctr; // current target
	float close_x, close_y;
	float heading, dang;
	float *localmap;
	int mapnx, mapny;
	bool t;
	t = false;

	mapnx = an->slammer->nx;
	mapny = an->slammer->ny;
	localmap = new float [mapnx*mapny];

	while (an->running)
	{
		// should we do something?
		an->anlock.lock();
		if (an->triggered)
		{
			t = true;
			an->triggered = false;
		}
		// but dont try to move if we are out of targets
		if (an->currtarget >= an->target_x.size()) t = false;
		an->anlock.unlock();
		if (t)
		{
			// we have been told to do stuff
			// grab position and target from autonav
			an->anlock.lock();
			an->computing = true;
			cx = an->cx;
			cy = an->cy;
			ca = an->ca;
			ctx = an->target_x[an->currtarget];
			cty = an->target_y[an->currtarget];
			ctr = an->target_r[an->currtarget];
			an->anlock.unlock();
			
			// make a local copy of the current map
			an->slammer->slam_mutex.lock();
			memcpy(localmap, an->slammer->map, mapnx*mapny*sizeof(float));
			an->slammer->slam_mutex.unlock();
			
			// solve for optimal path
			// ...
			
			// pick local target to aim at
			close_x = ctx;
			close_y = cty;
			
			// set hexapod speed and turning
			heading = atan2(close_y-cy, close_x-cx);
			dang = heading - ca;
			if (dang < -PI) dang += 2.*PI;
			if (dang > PI) dang -= 2.*PI;

			an->hex->hexlock.lock();
			an->hex->speed = 1.0;
			if (fabs(dang) > 0.05)
				an->hex->turning = 5.*dang/PI;
			else
				an->hex->turning = 0.0;
			an->hex->hexlock.unlock();
		} else {
			usleep(10000);
		}
	}

	delete [] localmap;
}

autonav::autonav ()
{
	//
}

void autonav::init (hexapod *hex0, slam *slammer0, float x, float y, float a)
{
	cx = x;
	cy = y;
	ca = a;
	currtarget = 0;
	currtarget_time = 0.0;
	lastsolve = getTime();
	// link to various other classes
	slammer = slammer0;
	hex = hex0;
	// run listening thread
	computing = false;
	triggered = false;
	running = true;
	listener = thread(autonav_loop, this);
}

void autonav::addTarget (float xpos, float ypos, float rad)
{
	anlock.lock();
	target_x.push_back(xpos);
	target_y.push_back(ypos);
	target_r.push_back(rad);
	anlock.unlock();
}

void autonav::solve (float currx, float curry, float currang)
{
	float dt;
	dt = getTime() - lastsolve;
	lastsolve = getTime();
	// check which target we are going for
	anlock.lock();
	if (sqrt(pow(currx-target_x[currtarget],2) + pow(curry-target_y[currtarget],2)) < 
			target_r[currtarget])
	{
		if (currtarget_time < AUTONAV_TARGET_MINTIME)
		{
			currtarget_time += dt;
		} else {
			currtarget_time = 0.0;
			currtarget++;
		}
	}
	cx = currx;
	cy = curry;
	ca = currang;
	triggered = true;
	anlock.unlock();
}

void autonav::close()
{
	running = false;
	listener.join();
}

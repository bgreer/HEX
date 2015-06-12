#include "/home/bgreer/PROJECTS/HEX/LIB_AUTONAV/autonav.h"

void autonav_loop (autonav *an)
{
	int ii, ind, dx, dy;
	float cx, cy, ca; // current position
	float ctx, cty, ctr; // current target
	float close_x, close_y;
	float heading, dang, scale, newscore, minf;
	float *localmap;
	int nx, ny, x0, y0, xt, yt;
	long steps;
	an_node *thisnode, *newnode;
	vector<an_node*> openset, closedset, path;
	uint16_t thisx, thisy, newx, newy;
	bool t, pathfound, inclosed, inopen;
	data_chunk *d;
	t = false;

	nx = an->slammer->nx;
	ny = an->slammer->ny;
	localmap = new float [nx*ny];
	scale = an->slammer->scale;

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
			t = false;
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
			memcpy(localmap, an->slammer->map_filt, nx*ny*sizeof(float));
			an->slammer->slam_mutex.unlock();
			
			// solve for optimal path
			x0 = nx/2 + cx/scale;
			y0 = ny/2 + cy/scale;
			xt = nx/2 + ctx/scale;
			yt = ny/2 + cty/scale;
			openset.push_back(new an_node(x0, y0));
			openset[openset.size()-1]->f_score = -localmap[x0*ny+y0]*scale*MAP_WEIGHT + 
				pow(pow(x0-xt,2)+pow(y0-yt,2),DIST_POW);
			pathfound = false;
			steps = 0;
			while (openset.size() > 0 && !pathfound && steps < 2000)
			{
				steps ++;
				// start with node in openset with lowest f_score
				minf = 1e9;
				for (ii=0; ii<openset.size(); ii++)
				{
					if (openset[ii]->f_score < minf)
					{
						minf = openset[ii]->f_score;
						ind = ii;
					}
				}
				thisx = openset[ind]->xpos;
				thisy = openset[ind]->ypos;
				// check for target
				if (thisx == xt && thisy == yt)
				{
					pathfound = true;
				}
				if (!pathfound)
				{
					// move node to closed set
					thisnode = openset[ind];
					openset.erase(openset.begin()+ind);
					closedset.push_back(thisnode);
					// loop through neighbors
					for (dx=-1; dx<=1; dx++)
					{
						for (dy=-1; dy<=1; dy++)
						{
							newx = thisx+dx;
							newy = thisy+dy;
							// check bounds
							if (newy>=0 && newy<ny && newx>=0 && newx<nx)
							{
								// look for in closed set
								inopen = false;
								inclosed = false;
								for (ii=0; ii<closedset.size(); ii++)
									if (closedset[ii]->xpos == newx && 
											closedset[ii]->ypos == newy)
										inclosed = true;
								for (ii=0; ii<openset.size(); ii++)
									if (openset[ii]->xpos == newx && 
											openset[ii]->ypos == newy)
										inopen = true;
								if (!inclosed && !inopen)
								{
									newscore = thisnode->g_score + pow(dx*dx+dy*dy,DIST_POW);
									newnode = new an_node(newx, newy);
									newnode->g_score = newscore;
									newnode->pathtracer = thisnode;
									newnode->f_score = newscore + 
										-localmap[newx*ny+newy]*scale*MAP_WEIGHT + 
										sqrt(pow(newx-xt,2)+pow(newy-yt,2));
									openset.push_back(newnode);
								}
							}
						}
					}
				}
			}
			cout << "reconstructing path after "<<steps<<" steps" << endl;
			// reconstruct optimal path
			// and pick local target to aim at
			thisnode = openset[ind];
			path.push_back(thisnode);
			close_x = (thisnode->xpos-nx/2)*scale;
			close_y = (thisnode->ypos-ny/2)*scale;
			d = new data_chunk('A');
			while (thisnode->xpos != x0 || thisnode->ypos != y0)
			{
				thisnode = thisnode->pathtracer;
				d->add(thisnode->xpos);
				d->add(thisnode->ypos);
				path.push_back(thisnode);
				if (sqrt(pow(thisnode->xpos-x0,2)+pow(thisnode->ypos-y0,2))*scale
						> AN_MIN_TARGET_DIST)
				{
					close_x = (thisnode->xpos-nx/2)*scale;
					close_y = (thisnode->ypos-ny/2)*scale;
				}
			}
			an->log->send(d);
			
			// clear sets
			for (ii=0; ii<openset.size(); ii++)
				delete openset[ii];
			openset.erase(openset.begin(), openset.end());
			for (ii=0; ii<closedset.size(); ii++)
				delete closedset[ii];
			closedset.erase(closedset.begin(), closedset.end());
			path.erase(path.begin(), path.end());
			
			// set hexapod speed and turning
			heading = atan2(close_y-cy, close_x-cx);
			dang = heading - ca;
			if (dang < -PI) dang += 2.*PI;
			if (dang > PI) dang -= 2.*PI;

			an->hex->hexlock.lock();
			an->hex->speed = 0.3;
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


void autonav::init (hexapod *hex0, slam *slammer0, logger *log0, 
		float x, float y, float a)
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
	log = log0;
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

// format of (ascii) target file is:
// xpos0	ypos0	radius0
// xpos1	ypos1	radius1
// ...
void autonav::loadTargets (string fname)
{
	int ii, num, ret;
	// old school file reading because I don't have internet right now
	// so I can't look up the c++ way
	FILE *fp;
	char instr[512];
	float f1, f2, f3;

	fp = fopen(fname.c_str(), "r");
	if (fp == NULL)
	{
		cout << "ERROR: could not open waypoint file: " << fname << endl;
		exit(-1);
	}
	// get number of lines
	num = 0;
	while (fgets(instr, 512, fp) != NULL)
		num ++;

	for (ii=0; ii<num; ii++)
	{
		ret = fscanf(fp, "%f\t%f\t%f\n", &f1, &f2, &f3);
		if (ret != 0)
		{
			cout << "ERROR: invalid waypoint at line " << (ii+1) << endl;
			exit(-1);
		}
		addTarget(f1,f2,f3);
	}
	
	fclose(fp);
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
			cout << "reached target!" << endl;
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

#include <iostream>
#include <vector>
#include <SDL.h>
#include "/home/bgreer/PROJECTS/HEX/LIB_SLAM/slam.h"
#include "/home/bgreer/PROJECTS/HEX/LIB_LOGGER/logger.h"

Uint32 makeColor (uint8_t r, uint8_t g, uint8_t b)
{
	return (((Uint32)r)<<16) | (((Uint32)g)<<8) | ((Uint32)b);
}

// the log file needs to have:
// - lidar scans
// - hex differential position
int main (int argc, char *argv[])
{
	double time, dt, mintime, maxtime, nexttime;
	unsigned char tag;
	int ii, num, index, nx_screen, ny_screen, ix, iy;
	int nx_slam, ny_slam, ind, ij;
	float scale;
	float value, init_x, init_y, init_ang;
	float currx, curry, currang, x, y, ang;
	bool done;
	Uint32 *pix;
	ifstream file;
	vector<data_chunk*> dlist;
	data_chunk *d;
	scan *s;
	slam slammer;
	SDL_Surface *screen;
	SDL_Rect full;

	nx_slam = 128;
	ny_slam = 128;
	scale = 10.0;
	nx_screen = 600;
	ny_screen = 600;
	dt = 0.1;
	init_x = 0.0;
	init_y = 0.0;
	init_ang = 0.0;
	x = init_x;
	y = init_y;
	ang = init_ang;

	full.x = 0;
	full.y = 0;
	full.w = nx_screen;
	full.h = ny_screen;

	if (argc < 2)
	{
		cout << "Usage: ./recon logfile" << endl;
		return -1;
	}

	// step 1: load log from file
	cout << "Loading log file.." << endl;
	mintime = 1e13;
	maxtime = 0;
	file.open(argv[1], ios::in | ios::binary);
	done = false;
	while (!done)
	{
		file.read(reinterpret_cast<char*>(&(time)), sizeof(double));
		if (time < mintime) mintime = time;
		if (time > maxtime) maxtime = time;
		file.read(reinterpret_cast<char*>(&(tag)), sizeof(unsigned char));
		file.read(reinterpret_cast<char*>(&(num)), sizeof(int));
		d = new data_chunk(tag);
		d->time = time;
		for (ii=0; ii<num; ii++)
		{
			file.read(reinterpret_cast<char*>(&(value)), sizeof(float));
			d->add(value);
		}
		dlist.push_back(d);
		if (file.eof()) done = true;
	}
	file.close();
	cout << "Done." << endl;
	cout << "Number of data entries: " << dlist.size() << endl;

	// step 2: set up slam
	cout << "Initializing SLAM.." << endl;
	slammer.init(nx_slam,ny_slam,scale);
	slammer.currx = init_x;
	slammer.curry = init_y;
	slammer.currang = init_x;
	slammer.setRegularization(0.3,0.3,0.3);
	cout << "Done." << endl;

	// step 3: set up SDL window
	if (SDL_Init(SDL_INIT_VIDEO) < 0)
	{
		cout << "ERROR in SDL init: " << SDL_GetError() << endl;
		return -1;
	}
	atexit(SDL_Quit);
	screen = SDL_SetVideoMode(nx_screen, ny_screen, 32, 0);
	pix = (Uint32*)screen->pixels;

	// begin stepping through log
	time = mintime;
	index = 0; // where we are in the dlist
	while (index < dlist.size() && time < maxtime+1.0)
	{		
		// increment time
		time += dt;
		usleep(dt*100000);
		// loop through entries leading up to this time
		cout << index << " " << dlist[index]->time << " " << time << endl;
		nexttime = dlist[index]->time;
		while (nexttime <= time)
		{
			// do something with dlist[index]
			if (dlist[index]->tag == 'I')
			{
				cout << "integrating at time " << dlist[index]->time << ", size=" << dlist[index]->num << endl;
				num = dlist[index]->num/2;
				s = new scan(num);
				for (ii=0; ii<num; ii++)
				{
					s->angle[ii] = dlist[index]->data[ii*2];
					s->dist[ii] = dlist[index]->data[ii*2+1];
				}
				slammer.integrate(s, init_x, init_y, init_ang);
				slammer.filter();
				delete s;
			} else if (dlist[index]->tag == 'S') {
				cout << "stepping at time " << dlist[index]->time << ", size=" << dlist[index]->num << endl;
				currx = dlist[index]->data[0];
				curry = dlist[index]->data[1];
				currang = dlist[index]->data[2];
				num = (dlist[index]->num-3)/2;
				s = new scan(num);
				for (ii=0; ii<num; ii++)
				{
					s->angle[ii] = dlist[index]->data[ii*2+3];
					s->dist[ii] = dlist[index]->data[ii*2+4];
				}
				slammer.submitScan(s, currx, curry, currang);
				delete s;
				while (slammer.computing)
					usleep(10000);
			} else if (dlist[index]->tag == 'P') {
				cout << "position at time " << dlist[index]->time << ", size=" << dlist[index]->num << endl;
				x = dlist[index]->data[0];
				y = dlist[index]->data[1];
				ang = dlist[index]->data[2];
			} else if (dlist[index]->tag == 'A') {
				cout << "autonav path at time " << dlist[index]->time << ", size=" << dlist[index]->num << endl;
				// autonav path
			}
			index++;
			if (index < dlist.size())
				nexttime = dlist[index]->time;
			else
				nexttime = maxtime+1.0;
		}

		// render results to screen
		SDL_FillRect(screen, &full, 0);
		for (iy=0; iy<ny_screen; iy++)
		{
			for (ix=0; ix<nx_screen; ix++)
			{
				ind = iy*nx_screen + ix;

				// slam map, fill screen
				ii = ix*nx_slam/nx_screen;
				ij = iy*ny_slam/ny_screen;
				value = min(-0.04*slammer.map_filt[ii*ny_slam+ij],0.5) + slammer.map[ii*ny_slam+ij];
				pix[ind] = makeColor((uint8_t)min(510.*value,255.),
														(uint8_t)max(0.,min(510.*value-127.,255.)),
														(uint8_t)max(0.,min(510.*value-255.,255.)));

				// plot position
				if (sqrt(pow((ix-nx_screen/2)*scale*nx_slam/nx_screen-x,2)
							+pow((iy-ny_screen/2)*ny_slam*scale/ny_screen-y,2))
						<= 2*scale) // in cm
					pix[ind] = makeColor(255,255,255);

			}
		}
		SDL_Flip(screen);
	}
	cout << "Done looping." << endl;

}

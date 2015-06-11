#include <iostream>
#include <vector>
#include <SDL.h>
#include "/home/bgreer/PROJECTS/HEX/LIB_SLAM/slam.h"
#include "/home/bgreer/PROJECTS/HEX/LIB_LOGGER/logger.h"

Uint32 makeColor (uint8_t r, uint8_t g, uint8_t b)
{
	return (((Uint32)r)<<16) | (((Uint32)r)<<8) | ((Uint32)b);
}

// the log file needs to have:
// - lidar scans
// - hex differential position
int main (int argc, char *argv[])
{
	double time, dt;
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
	file.open(argv[1], ios::in | ios::binary);
	done = false;
	while (!done)
	{
		file.read(reinterpret_cast<char*>(&(time)), sizeof(double));
		file.read(reinterpret_cast<char*>(&(tag)), sizeof(unsigned char));
		file.read(reinterpret_cast<char*>(&(num)), sizeof(int));
		d = new data_chunk(tag);
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

	// step 2: set up slam
	slammer.init(nx_slam,ny_slam,scale);
	slammer.currx = init_x;
	slammer.curry = init_y;
	slammer.currang = init_x;
	slammer.setRegularization(0.3,0.3,0.3);

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
	time = 0.0;
	index = 0; // where we are in the dlist
	while (index < dlist.size())
	{		
		// increment time
		time += dt;
		// loop through entries leading up to this time
		while (dlist[index]->time <= time)
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
			}
			index++;
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
				value = slammer.map[ii*ny_slam+ij];
				pix[ind] = makeColor(100,127-value*127,127+value*127);

				// plot position
				if (sqrt(pow((ix-nx_slam/2)*scale-x,2)+pow((iy-ny_slam/2)*scale-y,2)) 
						<= 4*scale)
					pix[ind] = makeColor(255,255,255);

			}
		}
	}

}

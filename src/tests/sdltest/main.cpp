#include <iostream>
#include <vector>
#include <SDL.h>
#include <random>
#include "panel.h"

using namespace std;
#define NUM_INTEGRATE 10
#define COLOR_BLACK 0
#define COLOR_WHITE 0xffffff
#define COLOR_01 0x009cff
#define COLOR_02 0xffc000
#define COLOR_03 0xff203a
#define COLOR_04 0x84ff20

class crect
{
public:
	float x1, x2, y1, y2;
	crect (int a, int b, int c, int d)
	{x1 = a; x2 = b; y1 = c; y2 = d;}
	// given position x0,y0 and direction tht,
	// how far along that line do you have to go to collide with this rect
	// if no collision, return 1000
	float cdist (float x0, float y0, float tht)
	{
		float x, y, r, ret;
		ret = 1000.;

		// wall 1
		r = (x1-x0)/cos(tht);
		y = y0+r*sin(tht);
		if (y > y1 && y < y2 && r > 0.0) ret = min(ret, r);
		// wall 2
		r = (x2-x0)/cos(tht);
		y = y0+r*sin(tht);
		if (y > y1 && y < y2 && r > 0.0) ret = min(ret, r);
		// wall 3
		r = (y1-y0)/sin(tht);
		x = x0+r*cos(tht);
		if (x > x1 && x < x2 && r > 0.0) ret = min(ret, r);
		// wall 4
		r = (y2-y0)/sin(tht);
		x = x0+r*cos(tht);
		if (x > x1 && x < x2 && r > 0.0) ret = min(ret, r);
		return ret;
	}
};

scan* newScan (float realx, float realy, float realang, default_random_engine *gen)
{
	int ii, ij, num, ik;
	float d1, d2, d3, d4, tht, r;
	float w1, w2;
	float x1, x2, y1, y2, x0, y0, dr, xc, yc, d;
	float x, y;
	normal_distribution<float> dist(0.0,1.0);
	scan *ret;

	vector<crect> obj;

	obj.push_back(crect(-210,-200,-100,80));
	obj.push_back(crect(200,210,-100,80));
	obj.push_back(crect(-210,-50,80,150));
	obj.push_back(crect(-50,50,150,200));
	obj.push_back(crect(50,210,80,150));
	obj.push_back(crect(-200,200,-110,-100));

	obj.push_back(crect(-20,20,-20,20));

	num = 360;
	ret = new scan(num);

	for (ii=0; ii<num; ii++)
	{
		tht = 2.*PI*ii/((float)num);
		ret->angle[ii] = tht;
		tht += realang;
		ret->weight[ii] = 1.0;

		// scan through objects, look for closest collision
		r = 1000.;
		for (ij=0; ij<obj.size(); ij++)
		{
			r = min(r, obj[ij].cdist(realx, realy, tht));
		}
/*
		d1 = min((100.-realy)/sin(tht), 1000.);
		if (d1 < 0.0) d1 = 1000.;
		d2 = min((-100.-realy)/sin(tht), 1000.);
		if (d2 < 0.0) d2 = 1000.;
		d3 = min((200.-realx)/cos(tht), 1000.);
		if (d3 < 0.0) d3 = 1000.;
		d4 = min((-200.-realx)/cos(tht), 1000.);
		if (d4 < 0.0) d4 = 1000.;

		r = min(min(d1, d2), min(d3, d4));
*/
		r *= (1.0 + dist(*gen)*0.01);
		ret->dist[ii] = r;
	}

	return ret;
}


int main (void)
{
	int ii;
	float time;
	bool quit;
	float xpos, ypos, apos;
	SDL_Event event;
	SDL_Rect rect_full;
	SDL_Surface *screen;
	panel_map *p_mapper;
	default_random_engine gen;
	normal_distribution<float> dist(0.0,1.0);
	scan *s;
	ofstream file;
	slam slammer(128,64,7.0);

	if (SDL_Init(SDL_INIT_VIDEO) < 0)
	{
		cout << "ERROR in SDL init: " << SDL_GetError() << endl;
		return -1;
	}
	atexit(SDL_Quit);

	screen = SDL_SetVideoMode(800, 480, 32, SDL_NOFRAME);// | SDL_FULLSCREEN);
	SDL_ShowCursor(0);
	rect_full.x = 0; rect_full.y = 0;
	rect_full.w = 800; rect_full.h = 480;

	slammer.setRegularization(1.0,1.0,0.1);
	for (ii=0; ii<NUM_INTEGRATE; ii++)
	{
		s = newScan(80., 0.0, 0.0, &gen);
		slammer.integrate(s, 80.0, 0.0, 0.0);
		delete s;
	}

	//p_mapper = new panel_map(540,220,258,258,COLOR_WHITE,&slammer);
	p_mapper = new panel_map(2,2,796,476,COLOR_01,&slammer);

	time = 0.0;
	quit = 0;
	file.open("poslog");
	while (!quit)
	{
		// real positions
		xpos = 80.*cos(time/20.);
		ypos = 70.*sin(time/20.);
		apos = 0.0;
		// generate scan to match
		s = newScan(xpos, ypos, apos, &gen);
//		s = newScan(70.*sin(time/20.),0,0,&gen);
//		slammer.step(s, slammer.currx, slammer.curry, slammer.currang);
		slammer.step(s, xpos+dist(gen), ypos+dist(gen), apos+dist(gen)*0.01);
//		slammer.integrate(s, 0.0, 0.0, 0.0);
//		slammer.integrate(s, 80.*cos(time/20.), 80.*sin(time/20.), time*0.0);
//		slammer.currx = 80.*cos(time/20.);
//		slammer.curry = 80.*sin(time/20.);
		delete s;

		file << time << "\t" << slammer.currx << "\t" << slammer.curry << "\t" << slammer.currang << endl;

		// clear screen
		SDL_FillRect(screen, &rect_full, 0);

		// draw to screen
		p_mapper->draw(screen);

		// refresh screen
		SDL_Flip(screen);

		// check for event
		while (SDL_PollEvent(&event))
		{
			switch (event.type)
			{
				case SDL_QUIT:
					quit = true;
					break;
				case SDL_KEYDOWN:
					quit = true;
					break;
			}
		}
		//SDL_Delay(10);
		time += 0.4;
	}
	file.close();

	return 0;
}

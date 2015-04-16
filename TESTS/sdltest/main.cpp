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

scan* newScan (float realx, float realy, float realang, default_random_engine *gen)
{
	int ii, ij, num, ik;
	float d1, d2, d3, d4, tht, r;
	float w1, w2;
	float x1, x2, y1, y2, x0, y0, dr, xc, yc, d;
	normal_distribution<float> dist(0.0,1.0);
	scan *ret;

	vector<float> wallx, wally, wallr;

	wallx.push_back(800.0);
	wally.push_back(0.0);
	wallr.push_back(500.0);

	wallx.push_back(0);
	wally.push_back(0);
	wallr.push_back(50.);

	num = 360;
	ret = new scan(num);


	for (ii=0; ii<num; ii++)
	{
		tht = 2.*PI*ii/((float)num);
		ret->angle[ii] = tht;
		tht += realang;
		ret->weight[ii] = 1.0;

		d1 = min((100.-realy)/sin(tht), 1000.);
		if (d1 < 0.0) d1 = 1000.;
		d2 = min((-100.-realy)/sin(tht), 1000.);
		if (d2 < 0.0) d2 = 1000.;
		d3 = min((200.-realx)/cos(tht), 1000.);
		if (d3 < 0.0) d3 = 1000.;
		d4 = min((-200.-realx)/cos(tht), 1000.);
		if (d4 < 0.0) d4 = 1000.;

		r = min(min(d1, d2), min(d3, d4));

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
	SDL_Event event;
	SDL_Rect rect_full;
	SDL_Surface *screen;
	panel_map *p_mapper;
	default_random_engine gen;
	scan *s;
	slam slammer(128,64,5.0);

	if (SDL_Init(SDL_INIT_VIDEO) < 0)
	{
		cout << "ERROR in SDL init: " << SDL_GetError() << endl;
		return -1;
	}
	atexit(SDL_Quit);

	screen = SDL_SetVideoMode(800, 480, 32, SDL_NOFRAME | SDL_FULLSCREEN);
	SDL_ShowCursor(0);
	rect_full.x = 0; rect_full.y = 0;
	rect_full.w = 800; rect_full.h = 480;

	slammer.setRegularization(0.01,0.01,0.01);
	for (ii=0; ii<NUM_INTEGRATE; ii++)
	{
		s = newScan(0.0, 0.0, 0.0, &gen);
		slammer.integrate(s, 0.0, 0.0, 0.0);
		delete s;
	}

	//p_mapper = new panel_map(540,220,258,258,COLOR_WHITE,&slammer);
	p_mapper = new panel_map(2,2,796,476,COLOR_01,&slammer);

	time = 0.0;
	quit = 0;
	while (!quit)
	{
		// generate scan to match
		s = newScan(100.*cos(time/20.), 50.*sin(time/20.), time*0.6, &gen);
		slammer.step(s, slammer.currx, slammer.curry, slammer.currang);
//		slammer.step(s, 150.*sin(time/22.), 50.*sin(time/10.), time*0.6);
//		slammer.integrate(s, 0.0, 0.0, 0.0);
//		slammer.integrate(s, 150.*sin(time/22.), 50.*sin(time/10.), time*0.6);
		delete s;



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

	return 0;
}

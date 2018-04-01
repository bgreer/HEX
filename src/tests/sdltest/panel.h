#include "/home/bgreer/PROJECTS/HEX/LIB_SLAM/slam.h"

class panel
{
public:
	SDL_Rect bounds, innerbounds;
	Uint32 bordercolor;
	
	panel (int x0, int y0, int w, int h, Uint32 b)
	{
		bounds.x = x0;
		bounds.y = y0;
		bounds.w = w;
		bounds.h = h;
		innerbounds.x = x0+1;
		innerbounds.y = y0+1;
		innerbounds.w = w-2;
		innerbounds.h = h-2;
		bordercolor = b;
	}

	void clear (SDL_Surface *screen)
	{
		SDL_FillRect(screen, &bounds, bordercolor);
		SDL_FillRect(screen, &innerbounds, 0);
	}

	void draw (SDL_Surface *screen)
	{
		clear(screen);
	}
};


class panel_map : public panel
{
public:
	int mapnx, mapny, mapn;
	float *map;
	mutex *maplock;
	slam *slammer;

	panel_map (int x0, int y0, int w, int h, Uint32 b, slam *s)
		: panel (x0, y0, w, h, b)
	{
		attachSLAM (s);
	}

	void draw (SDL_Surface *screen)
	{
		int ix, iy, ind, mx, my;
		float val;
		Uint32 *pix, color;

		clear(screen);
		
		// draw map to window
		pix = (Uint32*)screen->pixels;
		maplock->lock();
		for (ix=0; ix<innerbounds.w; ix++)
		{
			for (iy=0; iy<innerbounds.h; iy++)
			{
				ind = (iy+innerbounds.y)*800 + ix+innerbounds.x;
				// nearest neighbor
				mx = ix*mapnx/innerbounds.w;
				my = iy*mapny/innerbounds.h;
				val = map[mx*mapny + my];
				color = 0;
				if (val > 0.25) color = 0xbb0000;
				if (val > 0.50) color = 0x888800;
				if (val > 0.75) color = 0x44aa88;
				if (val ==1.00) color = 0x44ddff;
				pix[ind] = color;
			}
		}
		// plot location
		mx = (slammer->currx/slammer->scale + mapnx/2)*innerbounds.w/mapnx;
		my = (slammer->curry/slammer->scale + mapny/2)*innerbounds.h/mapny;
		for (ix=-2;ix<=2;ix++)
		{
			for (iy=-2; iy<=2; iy++)
			{
				if (withinBounds(mx+ix,my+iy,0,innerbounds.w,0,innerbounds.h))
				{
					ind = (my+iy+innerbounds.y)*800 + mx+ix+innerbounds.x;
					pix[ind] = 0xffffff;
				}
			}
		}
		for (ix=1; ix<15; ix++)
		{
			if (withinBounds(mx+ix*cos(slammer->currang),
						my+ix*sin(slammer->currang),0,innerbounds.w,0,innerbounds.h))
			{
				ind = (my-(int)(ix*sin(slammer->currang))+innerbounds.y)*800 + 
					mx+ix*cos(slammer->currang)+innerbounds.x;
				pix[ind] = 0xffffff;
			}
		}
		maplock->unlock();
	}

	void attachSLAM (slam *s)
	{
		slammer = s;
		mapnx = s->nx;
		mapny = s->ny;
		mapn = max(mapnx, mapny);
		map = s->map;
		maplock = &(s->maplock);
	}
};

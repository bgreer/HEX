#include "header.h"

// manual control

void initSDL (SDL_Screen *screen, SDL_Joystick *joy)
{
	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK) < 0)
	{
		cout << "ERROR in SDL init: " << SDL_GetError() << endl;
		return -1;
	}
	atexit(SDL_Quit);
	screen = SDL_SetVideoMode(800, 480, 32, SDL_NOFRAME | SDL_FULLSCREEN);
	SDL_ShowCursor(0);
	SDL_JoystickEventState(SDL_ENABLE);
	// sort out joystick control
	if (SDL_NumJoysticks()>0)
	{
		joy=SDL_JoystickOpen(0);
		if (joy)
		{
			cout << "Joystick Connected." << endl;
		} else {
			cout << "ERROR: could not connect to joystick!" << endl;
			return -1;
		}
	} else {
		cout << "ERROR: could not find joystick!" << endl;
		return -1;
	}

}

void updateManualControl (bool *quit, float *speed, float *turning, 
		float *height)
{
	float joyval;
	SDL_Event event;

	while (SDL_PollEvent(&event))
	{
		switch (event.type)
		{
			case SDL_JOYBUTTONDOWN: // start button
				if (event.jbutton.button == 7) *quit = true;
				break;
			case SDL_JOYAXISMOTION:
				joyval = -event.jaxis.value/32767.;
				if (event.jaxis.axis == 1) // L stick, yaxis
				{
					if (joyval > 0.1) *speed = 0.5*(joyval - 0.1);
					else if (joyval < -0.1) *speed = 0.5*(joyval + 0.1);
					else *speed = 0.0;
				}
				if (event.jaxis.axis == 2) // R stick, xaxis
				{
					if (joyval > 0.1) *turning = (joyval-0.1);
					else if (joyval < -0.1) *turning = (joyval+0.1);
					else *turning = 0.0;
				}
				if (event.jaxis.axis == 3) // R stick, yaxis
				{
					if (joyval > 0.1) *height = (joyval-0.1)*2.0;
					else if (joyval < -0.1) *height = (joyval+0.1)*2.0;
					else *height = 0.0;
				}
				break;
		}
	}
}

bool getButtonPress (int id, bool blocking)
{
	bool ret = false;
	SDL_Event event;

	while (blocking && !ret)
	{
		while (SDL_PollEvent(&event))
		{
			switch (event.type)
			{
				case SDL_JOYBUTTONDOWN:
					if (event.jbutton.button == id) ret = true;
					break;
			}
		}
		if (blocking) SDL_Delay(10);
	}

}

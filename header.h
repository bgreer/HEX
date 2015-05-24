#include <SDL.h>
#include "/home/bgreer/PROJECTS/HEX/LIB_SERIAL/serial.h"
#include "/home/bgreer/PROJECTS/HEX/LIB_HEXAPOD/hexapod.h"
#include "/home/bgreer/PROJECTS/HEX/LIB_LOGGER/logger.h"

using namespace std;

// code in a header? blasphemy!
double getTime()
{
	double ret;
	struct timeval tv;
	gettimeofday(&tv, NULL);
	ret = (tv.tv_sec) + tv.tv_usec*1e-6;
	return ret;
}


// function prototypes

// manual.cpp
void initSDL(SDL_Screen *screen, SDL_Joystick *joy);
bool getButtonPress(int id, bool blocking);
void updateManualControl (bool *quit, float *speed, float *turning, 
		float *height);

// actions.cpp
void performSafeStand(hexapod *hex, serial *ser);
void sendServoPositions(hexapod *hex, serial *ser);

#include <SDL.h>
#include "/home/bgreer/PROJECTS/HEX/LIB_SLAM/slam.h"
#include "/home/bgreer/PROJECTS/HEX/LIB_SERIAL/serial.h"
#include "/home/bgreer/PROJECTS/HEX/LIB_HEXAPOD/hexapod.h"
#include "/home/bgreer/PROJECTS/HEX/LIB_LOGGER/logger.h"

using namespace std;


// function prototypes

// manual.cpp
int initSDL (SDL_Surface *screen, SDL_Joystick *joy);
bool getButtonPress (int id, bool blocking);
void updateManualControl (bool *quit, float *speed, float *turning, 
		float *height);

// actions.cpp
scan* getLIDARData (serial *ser, bool blocking);
void setLIDARSpin (serial *ser, bool enabled);
void performSafeStand (hexapod *hex, serial *ser);
void sendServoPositions (hexapod *hex, serial *ser);
void disableServos (serial *ser);
void enableServos (serial *ser);

// monitor.cpp
double getTime();



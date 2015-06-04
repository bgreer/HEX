
// are we compiling with manual control?
#define MANUAL

#include "/home/bgreer/PROJECTS/HEX/LIB_AUTONAV/autonav.h"
#include "/home/bgreer/PROJECTS/HEX/LIB_SLAM/slam.h"
#include "/home/bgreer/PROJECTS/HEX/LIB_SERIAL/serial.h"
#include "/home/bgreer/PROJECTS/HEX/LIB_HEXAPOD/hexapod.h"
#include "/home/bgreer/PROJECTS/HEX/LIB_LOGGER/logger.h"
#ifdef MANUAL
#include <SDL.h>
#endif

using namespace std;


// function prototypes

// manual.cpp
#ifdef MANUAL
int initSDL (SDL_Surface *screen, SDL_Joystick *joy);
bool getButtonPress (int id, bool blocking);
void updateManualControl (bool *quit, float *speed, float *turning, 
		float *height);
#endif

// actions.cpp
scan* getLIDARData (serial *ser, bool blocking);
void setLIDARSpin (serial *ser, bool enabled);
void performSafeStand (hexapod *hex, serial *ser);
void sendServoPositions (hexapod *hex, serial *ser);
void disableServos (serial *ser);
void enableServos (serial *ser);

// monitor.cpp
double getTime();



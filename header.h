
// are we compiling with manual control?
//#define MANUAL
// print to stdout as well
#define DEBUG 1

#define LED_GREEN 17
#define LED_BLUE 18
#define LED_WHITE 41
#define BUTTON_2 16
#define BUTTON_3 20
#define BUTTON_4 19

#include "/home/bgreer/PROJECTS/HEX/LIB_AUTONAV/autonav.h"
#include "/home/bgreer/PROJECTS/HEX/LIB_SLAM/slam.h"
#include "/home/bgreer/PROJECTS/HEX/LIB_SERIAL/serial.h"
#include "/home/bgreer/PROJECTS/HEX/LIB_HEXAPOD/hexapod.h"
#include "/home/bgreer/PROJECTS/HEX/LIB_LOGGER/logger.h"
#include "tclap/CmdLine.h"
#ifdef MANUAL
#include <SDL.h>
#endif

using namespace std;


// function prototypes

// manual.cpp
#ifdef MANUAL
int initSDL (SDL_Surface *screen, SDL_Joystick *joy);
bool getRemoteButtonPress (int id, bool blocking);
void updateManualControl (bool *quit, float *speed, float *turning, 
		float *height);
#endif

// actions.cpp
bool confirmArbotixConnection (serial *ser, hexapod *hex);
scan* getLIDARData (serial *ser, bool blocking);
void setLIDARSpin (serial *ser, bool enabled);
void performSafeStand (hexapod *hex, serial *ser);
void performRaceFinish (hexapod *hex, serial *ser);
void sendServoPositions (hexapod *hex, serial *ser);
void disableServos (serial *ser);
void enableServos (serial *ser);

// monitor.cpp
double getTime();
void setLED (uint8_t id, bool value);
bool getButtonPress (uint8_t id, bool blocking);

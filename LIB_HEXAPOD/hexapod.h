#include <iostream>
#include <math.h>
#include "bezier2d.h"
#define PI 3.14159265
#define DEGTORAD 0.01745329251
#define RADTODEG 57.2957795131

#define MAX_SPEED 1.5
#define SS_DURATION 2.0
#define TURN_TOL 0.005
#define MAXITER 300
// position tolerance in cm
#define TOLERANCE 0.05
// angle to move things away from bounds by, in radians
#define ANGEPS 0.01

using namespace std;

/* servo IDs:
 *           back
 *	06 10 08 ---- 07 09 11 
 *	12 16 14 ---- 13 15 17
 *	18 04 02 ---- 01 03 05
 *           front
 *
 * convert to index:
 *  (LEG 5) 17 16 15 ---- 06 07 08 (LEG 2)
 *  (LEG 4) 14 13 12 ---- 03 04 05 (LEG 1)
 *  (LEG 3) 11 10 09 ---- 00 01 02 (LEG 0)
 *
 * tibia, femur, coxa ---- coxa, femur, tibia
 *
 * limits (ignoring intersection with other limbs):
 * 	coxa: 245 - 75
 * 	femur: 250 - 55
 * 	tibia: 215 - 40
 */

/* Coordinate system:
 *  Origin is bottom of main chassis, center horizontally
 *    Z OO==> Y
 *      ||
 *      \/ X
 */

enum Groups {GROUP_ALL, GROUP_TIBIA, GROUP_FEMUR, GROUP_COXA, 
			GROUP_RIGHT, GROUP_LEFT, 
			GROUP_FRONT, GROUP_MIDDLE, GROUP_BACK};

class hexapod
{
public:
	// both sets of angles are relative to leg root and leg angle
	float servoangle[18]; // in degrees, 0 to 300 or so (subject to more constraint)
	float angle[18]; // in radians, direction and offset corrected

	// hexapod body
	float length[3]; // length of coxa, femur, tibia
	float femurangle, tibiaangle;
	float angleub[3], anglelb[3]; // angle bounds
	float legpos[6][3]; // root of leg in xyz
	float legpos1[6][3]; // default resting position of leg
	float legang[6]; // root angle of leg

	// for walking
	float time, speed;
	float smoothspeed, fdf;
	float turning, smoothturning;
	float standheight;
	float sweepmodifier, speedmodifier, maxsweep;

	// for safestand
	float sstime;
	bool ssrunning;
	float ssx0[6], ssy0[6], ssz0[6]; // initial positions

	// dead-reckoning
	float dr_xpos, dr_ypos, dr_ang;

	// for walking
	bezier2d b2d_walk_up, b2d_walk_down;

	hexapod ();

	void safeStand ();
	void step (float dt);
	void setAngles ();
	void setServoAngles ();
	bool IKSolve (int leg, float *target);
	void FKSolve (int leg, float *angles, float *pos);
	void stand ();
	void sit ();

};

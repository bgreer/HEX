#include "/home/bgreer/PROJECTS/HEX/LIB_AXSERVO/axservo.h"

/* servo IDs:
 *           back
 *	06 10 08 ---- 07 09 11 
 *	12 16 14 ---- 13 15 17
 *	18 04 02 ---- 01 03 05
 *           front
 *
 * convert to index:
 *  17 16 15 ---- 06 07 08
 *  14 13 12 ---- 03 04 05
 *  11 10 09 ---- 00 01 02
 *
 * tibia, femur, coxa ---- coxa, femur, tibia
 *
 * limits (ignoring intersection with other limbs):
 * 	coxa: 245 - 75
 * 	femur: 250 - 55
 * 	tibia: 215 - 40
 */

enum Groups {GROUP_ALL, GROUP_TIBIA, GROUP_FEMUR, GROUP_COXA, 
			GROUP_RIGHT, GROUP_LEFT, 
			GROUP_FRONT, GROUP_MIDDLE, GROUP_BACK};

class hexapod
{
public:
	int idmap[18];
	axservo* joint[18];

	hexapod ()
	{
		int ii;

		idmap[0] = 1; idmap[1] = 3; idmap[2] = 5;
		idmap[3] = 13; idmap[4] = 15; idmap[5] = 17;
		idmap[6] = 7; idmap[7] = 9; idmap[8] = 11;
		idmap[9] = 2; idmap[10] = 4; idmap[11] = 18;
		idmap[12] = 14; idmap[13] = 16; idmap[14] = 12;
		idmap[15] = 8; idmap[16] = 10; idmap[17] = 6;
		
		// initialize
		ax12Init(1000000);
		for (ii=0; ii<18; ii++)
		{
			joint[ii] = new axservo(idmap[ii]);
			joint[ii]->setComplianceSlopes(64,64);
			if (ii<9) joint[ii]->reverse = true;
		}
		enableJoints();
		
	}

	void moveStand (float duration)
	{
		int ii;
		for (ii=0; ii<18; ii++)
		{
			if (ii % 3 == 0)
				joint[ii]->setPosition(150.);
			if (ii % 3 == 1)
				joint[ii]->setPosition(130.);
			if (ii % 3 == 2)
				joint[ii]->setPosition(150.);
		}
	}
	void moveSit (float duration)
	{
		int ii;
		for (ii=0; ii<18; ii++)
		{
			if (ii % 3 == 0)
				joint[ii]->setPosition(150.);
			if (ii % 3 == 1)
				joint[ii]->setPosition(250.);
			if (ii % 3 == 2)
				joint[ii]->setPosition(40.);
		}
	}

	void enableJoints ()
	{
		int ii;
		for (ii=0; ii<18; ii++)
			joint[ii]->setTorqueEnable(true);
	}

	void disableJoints ()
	{
		int ii;
		for (ii=0; ii<18; ii++)
			joint[ii]->setTorqueEnable(false);
	}
};

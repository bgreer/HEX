#include "header.h"

void sendServoPositions (hexapod *hex, serial *ser)
{
	int ii;
	float pos;
	packet *pack;

	pack = new packet(dsize, 'A', psize);
	pack->data[0] = 0x01; // set servo positions
	
	// package positions
	for (ii=0; ii<18; ii++)
	{
		pos = hex.servoangle[ii];
		memcpy(pack->data+1+(ii)*sizeof(float), 
				&pos, sizeof(float));
	}
	ser->send(pack);
	delete pack;
}


void performSafeStand (hexapod *hex, serial *ser)
{
	int ii;
	float pos;
	double lasttime, dt;
	packet *pack;

	pack = new packet(dsize, 'A', psize);
	pack->data[0] = 0x01; // set servo positions
	
	lasttime = getTime();
	// begin a safeStand operation
	hex->safeStand();
	while (hex->ssrunning)
	{
		dt = (getTime() - lasttime);
		lasttime = getTime();
		hex->step(dt);
		// package positions
		for (ii=0; ii<18; ii++)
		{
			pos = hex.servoangle[ii];
			memcpy(pack->data+1+ii*sizeof(float), 
					&pos, sizeof(float));
		}
		ser->send(pack);
		usleep(20*1000);
	}

	delete pack;
}

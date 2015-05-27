#include "header.h"

scan* getLIDARData (serial *ser, bool blocking)
{
	uint16_t d, ii;
	float fd;
	int num;
	scan *s;
	packet *pack, *recv;

	// first, check for a packet that's already waiting
	if ((recv = ser->recv('U', 0x02, false)) == NULL)
	{
		// no packet found, send request
		pack = new packet(16, 'D');
		pack->data[0] = 0x02; // command byte, lidar data request
		ser->send(pack, true);
		delete pack;
		if (blocking)
		{
			// wait for result
			while ((recv = ser->recv('U', 0x02, false)) == NULL)
				usleep(1000);
		} else {
			// no immediate data, no blocking
			// so no data updated
			return NULL;
		}
	}

	// we have a packet, parse it!
	/*
	num = 0;
	for (ii=0; ii<360; ii++)
	{
		memcpy(&d, &(recv->data[1+ii*2]), sizeof(uint16_t));
		fd = d*0.1; // turn into cm;
		// data culling
		if (fd >= 10.0) num++;
	}
	*/
	num = 360;
	s = new scan(num);
	num = 0;
	for (ii=0; ii<360; ii++)
	{
		memcpy(&d, &(recv->data[1+ii*2]), sizeof(uint16_t));
		fd = d*0.1; // turn into cm;
		// data culling
//		if (fd >= 10.0)
		{
			s->angle[num] = ii*PI/180.;
			s->dist[num] = fd;
			s->weight[num] = 1.0;
			num++;
		}
	}

	delete recv;

	return s;

}

void setLIDARSpin (serial *ser, bool enabled)
{
	packet *pack;

	pack = new packet(32, 'D');
	pack->data[0] = 0x01; // command byte, lidar enable/disable
	if (enabled)
		pack->data[1] = 0x01; // value: on
	else
		pack->data[1] = 0x00;
	pack->data[2] = 'U'; // return, udoo
	ser->send(pack, true);
	delete pack;
}

void sendServoPositions (hexapod *hex, serial *ser)
{
	int ii;
	float pos;
	packet *pack;

	pack = new packet(96, 'A');
	pack->data[0] = 0x01; // set servo positions
	
	// package positions
	for (ii=0; ii<18; ii++)
	{
		pos = hex->servoangle[ii];
		memcpy(pack->data+1+(ii)*sizeof(float), 
				&pos, sizeof(float));
	}
	ser->send(pack, true);
	delete pack;
}
void disableServos (serial *ser)
{
	int ii;
	packet *pack;
	pack = new packet(18, 'A');
	pack->data[0] = 0x03;
	for (ii=0; ii<18; ii++)
		pack->data[ii+1] = 0x00;

	ser->send(pack, true);
	delete pack;
}

void enableServos (serial *ser)
{
	int ii;
	packet *pack;
	pack = new packet(18, 'A');
	pack->data[0] = 0x03;
	for (ii=0; ii<18; ii++)
		pack->data[ii+1] = 0x01;

	ser->send(pack, true);
	delete pack;
}

void performSafeStand (hexapod *hex, serial *ser)
{
	int ii;
	float pos;
	double lasttime, dt;
	packet *pack;

	pack = new packet(96, 'A');
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
			pos = hex->servoangle[ii];
			memcpy(pack->data+1+ii*sizeof(float), 
					&pos, sizeof(float));
		}
		ser->send(pack, true);
		usleep(20*1000);
	}

	delete pack;
}

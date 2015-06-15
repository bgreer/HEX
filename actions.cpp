#include "header.h"

// the Arbotix-M might be off or initializing, so keep sending the request
// until you hear something back
bool confirmArbotixConnection (serial *ser, hexapod *hex)
{
	int attempts, ii;
	float pos;
	packet *pack, *pack2;

	pack = new packet(16, 'A');
	pack->data[0] = 0x05;
	pack->data[1] = 0x01;
	pack->data[2] = 'U';
	ser->send(pack, true);
	usleep(500*1000);

	pack2 = NULL;
	attempts = 0;
	while ((pack2 = ser->recv('U', 0x01, false)) == NULL && attempts < 10)
	{
		attempts ++;
		ser->send(pack, true);
		sleep(1);
	}
	delete pack;

	if (attempts==10) return false;

	// let the hex library know where the actual servos are right now
	for (ii=0; ii<18; ii++)
	{
		memcpy(&pos, pack2->data+1+ii*(sizeof(float)+sizeof(uint8_t)), 
				sizeof(float));
		hex->servoangle[ii] = pos;
	}
	hex->setAngles();
	delete pack2;


	return true;
}

scan* getLIDARData (serial *ser, bool blocking)
{
	uint16_t d, ii;
	uint8_t lidar_index;
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
			while ((recv = ser->recv('U', 0x02, true,0.5)) == NULL)
				ser->send(pack, true);

//			while ((recv = ser->recv('U', 0x02, false)) == NULL)
//				usleep(1000);
		} else {
			// no immediate data, no blocking
			// so no data updated
			return NULL;
		}
	}

	// we have a packet, parse it!
	num = 0;
	lidar_index = recv->data[1];
	for (ii=0; ii<40; ii++)
	{
		memcpy(&d, &(recv->data[2+ii*2]), sizeof(uint16_t));
		fd = d*0.1; // turn into cm;
		// data culling
		if (fd >= 10.0) num++;
	}
	s = new scan(num);
	num = 0;
	for (ii=0; ii<40; ii++)
	{
		memcpy(&d, &(recv->data[2+ii*2]), sizeof(uint16_t));
		fd = d*0.1; // turn into cm;
		// data culling
		if (fd >= 10.0)
		{
			s->angle[num] = (ii*9+lidar_index)*PI/180.;
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


void performRaceFinish (hexapod *hex, serial *ser)
{
	int ii;
	float pos;
	double lasttime, dt, inittime;
	packet *pack;

	pack = new packet(96, 'A');
	pack->data[0] = 0x01; // set servo positions
	
	inittime = getTime();
	lasttime = inittime;
	// first 2 seconds, slow down
	hex->speed = 0.0;
	hex->turning = 0.0;
	while (getTime() - inittime < 2.0)
	{
		dt = (getTime() - lasttime);
		lasttime = getTime();
		hex->step(dt);
		// send positions
		for (ii=0; ii<18; ii++)
		{
			pos = hex->servoangle[ii];
			memcpy(pack->data+1+ii*sizeof(float), 
					&pos, sizeof(float));
		}
		ser->send(pack, true);
		usleep(20*1000);
	}

	inittime = getTime();
	lasttime = inittime;
	// do a little dance!
	while (getTime()-inittime < 4.0)
	{
		dt = (getTime() - lasttime);
		lasttime = getTime();
		// oscillate once per second
		hex->standheight = 2.0*cos((lasttime - inittime)*PI);
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
	usleep(100*1000);

	delete pack;
}

#include <ax12.h>
#include "/home/bgreer/PROJECTS/HEX/LIB_AXSERVO/axservo.h"
#include "/home/bgreer/PROJECTS/HEX/LIB_PACKET/packet.h"

#define NUM_SERVOS 18
#define LOOP_DELAY 1

// TARGET: Arbotix-M controller
// the theme for communication here is:
//  do not speak unless spoken to

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

axservo *servo[NUM_SERVOS];
int idmap[NUM_SERVOS];
float temp[NUM_SERVOS], pos[NUM_SERVOS];
float avgtemp, maxtemp;

packet *currpacket;
int packetstart, headercount, psize, dsize, packetind;
unsigned char readbuffer[1024];
unsigned char buffer[10], tag, checksum;
bool packetloading;

// parse the current packet and set limits based on it
// setting all servo takes about 3.2 ms
void setPositions ()
{
	int ii;
	float pos;

	for (ii=0; ii<NUM_SERVOS; ii++)
	{
		memcpy(&pos, currpacket->buffer+PACKET_HEADER_SIZE+ii*sizeof(float), 
				sizeof(float));
		servo[ii]->setPosition(pos);
	}
}

void setLimits()
{

}

void sendData()
{
	int ii;
	float pos;
	packet *pack;

	pack = new packet(NUM_SERVOS*sizeof(float), 'D', 64);
	for (ii=0; ii<NUM_SERVOS; ii++)
	{
		pos = servo[ii]->getPosition();
		memcpy(pack->buffer+PACKET_HEADER_SIZE+ii*sizeof(float), 
				&pos, sizeof(float));
	}
	Serial.write(pack->buffer, pack->packet_size);
	delete pack;
}

void parsePacket ()
{
	int ii;
	float pos;
	unsigned long t0, t1;
	// TODO: use checksum

	if (!currpacket->verify()) 
	{
		Serial.println("CHECKSUM ERROR");
		return;
	}

	switch (currpacket->getTag())
	{
		case 'D': // get servo data
			sendData();
			break;
		case 'S': // set servo positions
			setPositions();
			break;
		case 'L': // set servo limits
			setLimits();
			break;
	}
}

void setup ()
{
	int ii;
	unsigned char trash;
	Serial.begin(115200);
	Serial.setTimeout(1);
	randomSeed(analogRead(0));

	ax12Init(1000000);

	// because 1) i dont like the default servo positions
	// and 2) i messed them up anyways
	// remap here.
	idmap[0] = 1; idmap[1] = 3; idmap[2] = 5;
	idmap[3] = 13; idmap[4] = 15; idmap[5] = 17;
	idmap[6] = 7; idmap[7] = 9; idmap[8] = 11;
	idmap[9] = 2; idmap[10] = 4; idmap[11] = 18;
	idmap[12] = 14; idmap[13] = 16; idmap[14] = 12;
	idmap[15] = 8; idmap[16] = 10; idmap[17] = 6;

	for (ii=0; ii<NUM_SERVOS; ii++)
	{
		servo[ii] = new axservo(idmap[ii]);
		// for nice smooth motion
		servo[ii]->setComplianceSlopes(64,64);
		// half the body is reversed
		if (ii<9 && !(ii%3==0)) servo[ii]->reverse = true;
		// stiffen
		servo[ii]->setTorqueEnable(true);
		// set servos to 'fast' mode
		// only ack when asked to
		servo[ii]->setReturnLevel(1);
		// delay 10us between commands
		servo[ii]->setReturnDelay(10);
		// make local delay the same
		servo[ii]->SERVO_DELAY = 10;
	}

	packetstart = 0;
	packetloading = false;
	delay(50);
	// setup done, ready to act
}

void loop ()
{
	int ii, nbytes, num;
	unsigned char inByte;
	unsigned long t0, t1;

	// listen for packets
	nbytes = 0;
	while (Serial.available() && nbytes < 1024)
	{
		readbuffer[nbytes] = Serial.read();
		nbytes++;
	}

	if (nbytes>0)
	{
		for (ii=0; ii<nbytes; ii++)
		{
		inByte = readbuffer[ii];
//		Serial.print(ii);
	//	Serial.print(" ");
		//Serial.println(inByte,HEX);
		if (inByte == 0x11) packetstart ++;
		else packetstart = 0;
		if (packetstart == 3)
		{
			if (packetloading) // we were already loading a packet
			{
				delete currpacket;
				packetloading = false;
			}
			packetloading = true;
			headercount = 0;
			packetstart = 0;
		} else if (packetloading) {
			if (headercount < 10)
			{
				buffer[headercount] = inByte;
				headercount ++;
				if (headercount == 10) // parse header
				{
					memcpy(&psize, buffer, 4);
					memcpy(&dsize, buffer+4, 4);
					memcpy(&tag, buffer+8, 1);
					memcpy(&checksum, buffer+9, 1);
					if (psize > 2048) psize = 2048;
					if (dsize > psize) dsize = psize;
					currpacket = new packet(dsize, tag, psize);
					currpacket->buffer[PACKET_HEADER_CHECKSUM] = checksum;
					packetind = 0;
				}
			} else {
				currpacket->buffer[PACKET_HEADER_SIZE+packetind] = inByte;
				packetind ++;
				if (packetind+headercount == dsize) // done loading
				{
					parsePacket();
					delete currpacket;
					packetloading = false;
					packetind = 0;
				}
			}
		}
		}

	}
	
	delay(LOOP_DELAY);
}

#include <ax12.h>
#include "/home/bgreer/PROJECTS/HEX/LIB_AXSERVO/axservo.h"
#include "/home/bgreer/PROJECTS/HEX/LIB_PACKET/packet.h"

#define READ_BUFFER_SIZE 128
#define PREBUFF_SIZE 1024
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

packet *currpacket;
int packetstart, headercount, psize, dsize, packetind;
unsigned char readbuffer[READ_BUFFER_SIZE], prebuff[PREBUFF_SIZE];
unsigned char sendbuff[128];
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
		memcpy(&pos, currpacket->data+1+ii*sizeof(float), 
				sizeof(float));
		servo[ii]->setPosition(pos);
	}
}

void setLimits()
{
	int ii;
	float ang1, ang2;

	for (ii=0; ii<NUM_SERVOS; ii++)
	{
		memcpy(&ang1, currpacket->data+1+ii*2*sizeof(float), 
				sizeof(float));
		memcpy(&ang2, currpacket->data+1+(ii*2+1)*sizeof(float), 
				sizeof(float));
		servo[ii]->setRotationLimits(ang1, ang2);
	}
}

void setTorqueEnable ()
{
	int ii;
	bool enable;

	for (ii=0; ii<NUM_SERVOS; ii++)
	{
		memcpy(&enable, currpacket->data+1+ii, 1);
		servo[ii]->setTorqueEnable(enable);
	}
}

void sendData(unsigned char info, unsigned char dest)
{
	int ii;
	float value;
	uint8_t err;
	packet *pack;

	// space for info and possible error codes
	pack = new packet(NUM_SERVOS*(sizeof(float) + sizeof(uint8_t))+1, 
			dest, 0, sendbuff);
	pack->data[0] = info; // repeat back what info this is

	for (ii=0; ii<NUM_SERVOS; ii++)
	{
		value = 0.0;
		switch (info)
		{
			case 0x01:
				value = servo[ii]->getPosition();
				break;
			case 0x02:
				value = servo[ii]->getTemperature();
				break;
			case 0x03:
				value = servo[ii]->getSpeed();
				break;
			case 0x04:
				value = servo[ii]->getLoad();
				break;
			case 0x05:
				value = servo[ii]->getVoltage();
				break;
		}
		err = servo[ii]->getError();
		memcpy(pack->data+1+ii*(sizeof(float)+sizeof(uint8_t)), 
				&value, sizeof(float));
		memcpy(pack->data+1+ii*(sizeof(float)+sizeof(uint8_t))+sizeof(float),
				&err, sizeof(uint8_t));
	}
	pack->setChecksum();
	Serial.write(pack->buffer, *(pack->p_packet_size));
	delete pack;
	memset(sendbuff, 0x00, 128);
}

void parsePacket ()
{
	int ii;
	float pos;
	unsigned long t0, t1;

	if (!currpacket->verify()) 
	{
//		Serial.println("CHECKSUM ERROR");
		return;
	}

	if (currpacket->getTag() == 'A') // this is the destination
	{
		// look at first byte of data to decide what action to take
		switch (currpacket->data[0])
		{
			case 0x01: // set servo positions
				setPositions();
				break;
			case 0x02: // set servo limits
				setLimits();
				break;
			case 0x03: // set torque enable / disable
				setTorqueEnable();
				break;
			case 0x04: // set compliance slopes
				// TODO
				break;
			case 0x05: // request data
				sendData(currpacket->data[1], currpacket->data[2]);
				break;
			default:
				break;
		}
	} else if (currpacket->getTag() == 'D') { // send to due
		Serial.write(currpacket->buffer, *(currpacket->p_packet_size));
	} else if (currpacket->getTag() == 'U') { // send to udoo
		Serial.write(currpacket->buffer, *(currpacket->p_packet_size));
	} else if (currpacket->getTag() == 'T') { // test packet, ignore
		//
	}

	// currpacket will be deleted after this function exits
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
		// keep servos slack
		servo[ii]->setTorqueEnable(false);
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
	while (Serial.available() && nbytes < READ_BUFFER_SIZE)
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
				memset(prebuff, 0x00, PREBUFF_SIZE);
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
					currpacket = new packet(dsize, tag, psize, prebuff);
					*(currpacket->p_checksum) = checksum;
					packetind = 0;
				}
			} else {
				currpacket->buffer[PACKET_HEADER_SIZE+packetind] = inByte;
				packetind ++;
				if (packetind+headercount == dsize) // done loading
				{
					parsePacket();
					delete currpacket;
					memset(prebuff, 0x00, PREBUFF_SIZE);
					packetloading = false;
					packetind = 0;
				}
			}
		}
		}

	}
	
	delay(LOOP_DELAY);
}

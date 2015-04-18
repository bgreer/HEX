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
		memcpy(&pos, currpacket->data+ii*sizeof(float), 
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
	float value;
	uint8_t flag;
	packet *pack;

	// determines what data to collect
	flag = (uint8_t)currpacket->buffer[PACKET_HEADER_SIZE];

	pack = new packet(NUM_SERVOS*sizeof(float)+1, 'E', 0, sendbuff);
	pack->buffer[PACKET_HEADER_SIZE] = flag;
	for (ii=0; ii<NUM_SERVOS; ii++)
	{
		value = 0.0;
		switch (flag)
		{
			case 1:
				value = servo[ii]->getPosition();
				break;
			case 2:
				value = servo[ii]->getTemperature();
				break;
			case 3:
				value = servo[ii]->getSpeed();
				break;
			case 4:
				value = servo[ii]->getLoad();
				break;
			case 5:
				value = servo[ii]->getVoltage();
				break;
		}
		memcpy(pack->buffer+PACKET_HEADER_SIZE+1+ii*sizeof(float), 
				&value, sizeof(float));
	}
	Serial.write(pack->buffer, *(pack->p_packet_size));
	delete pack;
	memset(sendbuff, 0x00, 128);
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
		case 'L': // set servo limits
			setLimits();
			break;
		case 'R': // repeat back as a test
			currpacket->setTag('T');
			Serial.write(currpacket->buffer, *(currpacket->p_packet_size));
			break;
		case 'S': // set servo positions
			setPositions();
			break;
		case 'T': // test message, ignore
			break;
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

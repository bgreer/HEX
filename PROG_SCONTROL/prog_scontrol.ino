#include <ax12.h>
#include "/home/bgreer/PROJECTS/HEX/LIB_AXSERVO/axservo.h"
#include "/home/bgreer/PROJECTS/HEX/LIB_PACKET/packet.h"

#define NUM_SERVOS 18
#define LOOP_DELAY 1

// TARGET: Arbotix-M controller

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

long checkTemp, checkPos; // how often to check on servos
long prevcheckTemp, prevcheckPos, currtime;

packet *currpacket;
int packetstart, headercount, psize, dsize, packetind;
unsigned char readbuffer[1024];
unsigned char buffer[9], tag;
bool packetloading;

void parsePacket ()
{
	int ii;
	float pos;
	unsigned long t0, t1;

	Serial.println("a");
	// setting all servo takes about 3.2 ms
	//t0 = micros();
	// do something with currpacket
	if (currpacket->getTag() == 'S')
	{
		servo[0]->setPosition(100.+random(100));
		for (ii=0; ii<18; ii++)
		{
			memcpy(&pos, currpacket->buffer+PACKET_HEADER_SIZE+ii*sizeof(float), sizeof(float));
			if (fabs(pos - 150.) < 30.)
				servo[ii]->setPosition(pos);
		}
	}
	//t1 = micros();
	//Serial.print("pp: ");
	Serial.println(t1-t0);
}

void setup ()
{
	int ii;
	unsigned char trash;
	Serial.begin(115200);
	Serial.setTimeout(1);
	randomSeed(analogRead(0));

	checkTemp = 0;
	checkPos = 0;

	ax12Init(1000000);
	
	idmap[0] = 1; idmap[1] = 3; idmap[2] = 5;
	idmap[3] = 13; idmap[4] = 15; idmap[5] = 17;
	idmap[6] = 7; idmap[7] = 9; idmap[8] = 11;
	idmap[9] = 2; idmap[10] = 4; idmap[11] = 18;
	idmap[12] = 14; idmap[13] = 16; idmap[14] = 12;
	idmap[15] = 8; idmap[16] = 10; idmap[17] = 6;

	for (ii=0; ii<NUM_SERVOS; ii++)
	{
		servo[ii] = new axservo(idmap[ii]);
		servo[ii]->setComplianceSlopes(64,64);
		if (ii<9) servo[ii]->reverse = true;
		servo[ii]->setTorqueEnable(true);
		servo[ii]->setReturnLevel(1);
		servo[ii]->setReturnDelay(10);
		servo[ii]->SERVO_DELAY = 10;
		Serial.println(servo[ii]->getReturnDelay());
//		servo[ii]->setPosition(150.);
	}
	currtime = millis();
	prevcheckPos = currtime;
	prevcheckTemp = currtime;

	packetstart = 0;
	packetloading = false;
	delay(100);
}

void loop ()
{
	int ii, nbytes, num;
	unsigned char inByte;
	unsigned long t0, t1;

	// listen for packets
	nbytes = 0;
	/*
	while ((num=Serial.available()) && nbytes < 1024-64)
	{
		Serial.readBytes((char*)(readbuffer+nbytes),1024);
//		Serial.println(num);
		nbytes += num;
	}
	*/
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
			if (headercount < 9)
			{
				buffer[headercount] = inByte;
				headercount ++;
				if (headercount == 9) // parse header
				{
					memcpy(&psize, buffer, 4);
					memcpy(&dsize, buffer+4, 4);
					memcpy(&tag, buffer+8, 1);
					if (psize > 2048) psize = 2048;
					if (dsize > psize) dsize = psize;
					currpacket = new packet(dsize, tag, psize);
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
	

	// check on various things
	currtime = millis();
	if (currtime - prevcheckPos > checkPos && checkPos > 0)
	{
		for (ii=0; ii<NUM_SERVOS; ii++)
			pos[ii] = servo[ii]->getPosition();
		prevcheckPos = currtime;
	}
	if (currtime - prevcheckTemp > checkTemp && checkTemp > 0)
	{
		avgtemp = 0.0;
		maxtemp = 0.0;
		for (ii=0; ii<NUM_SERVOS; ii++)
		{
			temp[ii] = servo[ii]->getTemperature();
			avgtemp += temp[ii];
			if (temp[ii] > maxtemp) maxtemp = temp[ii];
		}
		avgtemp /= (float)NUM_SERVOS;
		prevcheckTemp = currtime;
	}
	delay(LOOP_DELAY);
}

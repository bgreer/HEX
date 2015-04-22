#include "/home/bgreer/PROJECTS/HEX/LIB_PACKET/packet.h"

#define READ_BUFFER_SIZE 128
#define PREBUFF_SIZE 1024
#define LOOP_DELAY 1

// TARGET: UDOO Arduin Due

packet *currpacket;
int packetstart, headercount, psize, dsize, packetind;
unsigned char buffer[10], tag, checksum;
bool packetloading;
unsigned char readbuffer[READ_BUFFER_SIZE], prebuff[PREBUFF_SIZE];


void parsePacket ()
{
	int ii;
	float pos;
	unsigned long t0, t1;

	if (!currpacket->verify()) 
	{
		Serial.println("CHECKSUM ERROR");
		return;
	}

	switch (currpacket->getTag())
	{
		case 'R': // repeat back as a test
			currpacket->setTag('T');
			Serial.write(currpacket->buffer, *(currpacket->p_packet_size));
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
	Serial.begin(115200); // connection to i.MX6 processor
	Serial.setTimeout(1);
	Serial1.begin(115200); // arbotix-m, servo controller
	//Serial2.begin(115200); // teensy 3.1, lidar controller
	randomSeed(analogRead(0));


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
				if (packetind == dsize) // done loading
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

#include "/home/bgreer/PROJECTS/HEX/LIB_PACKET/packet.h"

#define READ_BUFFER_SIZE 128
#define PREBUFF_SIZE 1024
#define LOOP_DELAY 1

// TARGET: UDOO Arduin Due


// define a helping class
class ser_construct
{
public:
	HardwareSerial *ser; // which hardware serial port to use
	packet *pack;
	unsigned char readbuffer[READ_BUFFER_SIZE], prebuff[PREBUFF_SIZE];
	int packetstart, headercount;
	int psize, dsize, packetind;
	unsigned char buffer[10], tag, checksum;
	bool packetloading;

	ser_construct () {}
	void init (HardwareSerial *s)
	{
		ser = s;
		packetstart = 0;
		packetloading = false;
	}
};

ser_construct ser0, ser1;

void parsePacket (packet *pack)
{
	int ii;
	float pos;
	unsigned long t0, t1;

	if (!pack->verify()) 
	{
//		Serial.println("CHECKSUM ERROR");
	//	return;
	}

	if (pack->getTag() == 'D') // we are the target!
	{
		
	} else if (pack->getTag() == 'U') { // send to udoo
		Serial.write(pack->buffer, *(pack->p_packet_size));
	} else if (pack->getTag() == 'A') { // send to arbotix-m
		Serial1.write(pack->buffer, *(pack->p_packet_size));
	} else if (pack->getTag() == 'T') { // test message, disregard

	}

	// ser0_packet will be deleted after this function exits
}

void setup ()
{
	int ii;
	unsigned char trash;
	Serial.begin(115200); // connection to i.MX6 processor
	Serial.setTimeout(1);
	Serial1.begin(115200); // arbotix-m, servo controller
	Serial1.setTimeout(1);
	//Serial2.begin(115200); // teensy 3.1, lidar controller
	randomSeed(analogRead(0));

	// set up serial constructs
	ser0.init(&Serial);
	ser1.init(&Serial1);

	delay(50);
	// setup done, ready to act
}

void checkSerial (ser_construct *s)
{
	int ii, nbytes, num;
	unsigned char inByte;
	unsigned long t0, t1;

	// listen for packets
	nbytes = 0;
	while (s->ser->available() && nbytes < READ_BUFFER_SIZE)
	{
		s->readbuffer[nbytes] = s->ser->read();
		nbytes++;
	}

	if (nbytes>0)
	{
		for (ii=0; ii<nbytes; ii++)
		{
		inByte = s->readbuffer[ii];
		if (inByte == 0x11) s->packetstart ++;
		else s->packetstart = 0;
		if (s->packetstart == 3)
		{
			if (s->packetloading) // we were already loading a packet
			{
				delete s->pack;
				memset(s->prebuff, 0x00, PREBUFF_SIZE);
				s->packetloading = false;
			}
			s->packetloading = true;
			s->headercount = 0;
			s->packetstart = 0;
		} else if (s->packetloading) {
			if (s->headercount < 10)
			{
				s->buffer[s->headercount] = inByte;
				s->headercount ++;
				if (s->headercount == 10) // parse header
				{
					memcpy(&(s->psize), s->buffer, 4);
					memcpy(&(s->dsize), s->buffer+4, 4);
					memcpy(&(s->tag), s->buffer+8, 1);
					memcpy(&(s->checksum), s->buffer+9, 1);
					if (s->psize > 2048) s->psize = 2048;
					if (s->dsize > s->psize) s->dsize = s->psize;
					s->pack = new packet(s->dsize, s->tag, s->psize, s->prebuff);
					*(s->pack->p_checksum) = s->checksum;
					s->packetind = 0;
				}
			} else {
				s->pack->buffer[PACKET_HEADER_SIZE+s->packetind] = inByte;
				s->packetind ++;
				if (s->packetind == s->dsize) // done loading
				{
					parsePacket(s->pack);
					delete s->pack;
					memset(s->prebuff, 0x00, PREBUFF_SIZE);
					s->packetloading = false;
					s->packetind = 0;
				}
			}
		}
		}

	}

}

void loop ()
{

	checkSerial(&ser0); // check comms with i.mx6
	checkSerial(&ser1); // check comms with arbotix-m
	delay(LOOP_DELAY);
}

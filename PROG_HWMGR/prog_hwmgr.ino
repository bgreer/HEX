#include "/home/bgreer/PROJECTS/HEX/LIB_PACKET/packet.h"

#define READ_BUFFER_SIZE 128
#define PREBUFF_SIZE 1024
#define LOOP_DELAY 1

#define MOTOR_MAXVAL 150
#define MOTOR_MINVAL 50
#define MOTOR_PIN 8
#define MOTOR_GUESSVAL 100

// PID tuning params
#define PID_P 0.5
#define PID_I 0.1
#define PID_D 0.1

// TARGET: UDOO Arduin Due

// for PID motor controller
uint8_t motorval;
float currspeed, targetspeed;
float interror, lasterror;
uint8_t lidar_packet[22]; // for xv-11 lidar packet
int lidar_packet_index;
float lidar_dist[360], lidar_strength[360];
uint32_t lastpidupdate;

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
void computePID ()
{
	float error, deriv, dt;
	dt = (millis()-lastpidupdate)*0.001;

	error = targetspeed - currspeed;
	interror += error*dt;
	if (dt > 0.0)
		deriv = (error-lasterror)/dt;
	else
		deriv = 0.0;


	motorval = MOTOR_GUESSVAL;
	motorval += PID_P * error;
	motorval += PID_I * interror;
	motorval -= PID_D * deriv;
	
	lasterror = error;

	// make sure it stays in range
	if (motorval > MOTOR_MAXVAL) motorval = MOTOR_MAXVAL;
	if (motorval < MOTOR_MINVAL) motorval = MOTOR_MINVAL;
	// update the pwm control
	analogWrite(MOTOR_PIN, motorval);
	lastpidupdate = millis();
}

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
void lidar_parsePacket()
{
  int ii, ang;
  uint32_t chk32;
  uint16_t temp, chk_expected;
  uint16_t pos_offset;
  uint8_t invalid[4], warning[4];
  float speed, dist[4], signal[4], tht;
  float val, x, y;
  
  // first, check checksum
  chk32 = 0;
  for (ii=0; ii<10; ii++)
  {
    temp = (uint16_t)lidar_packet[ii*2] + (((uint16_t)lidar_packet[ii*2+1])<<8);
    chk32 = (chk32 << 1) + temp;
  }
  chk32 = (chk32 & 0x7fff) + ((chk32 >> 15) & 0x7fff);
  chk_expected = lidar_packet[20] | ((lidar_packet[21])<<8);
  
  // proceed if checksum is valid
  if (chk32 == chk_expected)
  {
		// info for this entire lidar packet
    pos_offset = (lidar_packet[1]-0xA0) * 4;
    currspeed = ((uint16_t)lidar_packet[2] + (((uint16_t)lidar_packet[3])<<8))/64.;
		if (millis()-lastpidupdate > 250)
			computePID();
    
		// info for each of the four measurements
    for (ii=0; ii<4; ii++)
    {
      dist[ii] = ((uint16_t)lidar_packet[4+ii*4] + 
        (((uint16_t)(lidar_packet[5+ii*4] & 0x3f))<<8))/1000.;
      invalid[ii] = (lidar_packet[5] & 0x80)>>7;
      warning[ii] = (lidar_packet[5] & 0x40)>>6;
      signal[ii] = (uint16_t)lidar_packet[6+ii*4] + 
        (((uint16_t)(lidar_packet[7+ii*4]))<<8);
    }
  }
}

void setup ()
{
	pinMode(MOTOR_PIN, OUTPUT);
	analogWrite(MOTOR_PIN, 0);
	Serial.begin(115200); // connection to i.MX6 processor
	Serial.setTimeout(1);
	Serial1.begin(115200); // arbotix-m, servo controller
	Serial1.setTimeout(1);
	Serial2.begin(115200); // XV-11 LIDAR unit
	randomSeed(analogRead(0));

	// set up serial constructs
	ser0.init(&Serial);
	ser1.init(&Serial1);

	// setup done, ready to act
	lidar_packet_index = 0;
	interror = 0.0;
	currspeed = 0.0;
	targetspeed = 320.0; // in units of 64th of an rpm (so 5 rpm)
	lastpidupdate = millis();
	analogWrite(MOTOR_PIN, MOTOR_GUESSVAL);
	delay(150);
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

void checkLidar ()
{
  byte in;
  
  while (Serial2.available())
  {
    in = Serial2.read();
    if (in==0xFA)
    {
      lidar_packet[0] = 0xFA;
      lidar_packet_index = 1;
    } else if (lidar_packet_index > 0)
    {
      lidar_packet[lidar_packet_index] = in;
      lidar_packet_index++;
      if (lidar_packet_index==22)
      {
        lidar_parsePacket();
        lidar_packet_index = 0;
      }
    }
  }
}

void loop ()
{

	checkSerial(&ser0); // check comms with i.mx6
	checkSerial(&ser1); // check comms with arbotix-m
	checkLidar(); // check comms with lidar unit
	delay(LOOP_DELAY);

}

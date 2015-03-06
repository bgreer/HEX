#include "/home/ubuntu/PROJECTS/HEX/LIB_PACKET/packet.h"

int parsePacket();

uint8_t packet2[22]; // for xv-11 lidar packet
int packet_index;
float avgspeed;
int count, loc, lastloc;
packet *pack;

// TODO list
// use packet library
// include signal strength in packet
// feedback loop for motor speed
// timing tests
// incorporate imu, servos

void setup ()
{
  Serial.begin(230400);
  Serial1.begin(115200); // connected to xv-11 lidar module
  packet_index = 0;
  loc = 0;
  lastloc = 0;
	pack = new packet((360*2+1)*sizeof(float), 'L', 1024);
	count = 0;
	avgspeed = 0.0;
}

void loop ()
{
  byte in;
  
  while (Serial1.available())
  {
    in = Serial1.read();
    if (in==0xFA)
    {
      packet2[0] = 0xFA;
      packet_index = 1;
    } else if (packet_index > 0)
    {
      packet2[packet_index] = in;
      packet_index++;
      if (packet_index==22)
      {
        loc = parsePacket();
        packet_index = 0;
        if (count > 360)
        {
          // send buffer
          Serial.write(pack->buffer, sizeof(char)*pack->packet_size);
					// TODO: clear packet?
          count = 0;
        }
        lastloc = loc;
      }
    }
  }
}

int parsePacket()
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
    temp = (uint16_t)packet2[ii*2] + (((uint16_t)packet2[ii*2+1])<<8);
    chk32 = (chk32 << 1) + temp;
  }
  chk32 = (chk32 & 0x7fff) + ((chk32 >> 15) & 0x7fff);
  chk_expected = packet2[20] | ((packet2[21])<<8);
  
  // proceed if checksum is valid
  if (chk32 == chk_expected)
  {
		// info for this entire lidar packet
    pos_offset = (packet2[1]-0xA0) * 4;
    speed = ((uint16_t)packet2[2] + (((uint16_t)packet2[3])<<8))/64.;
		avgspeed = 0.9*avgspeed + 0.1*speed;
    
		// info for each of the four measurements
    for (ii=0; ii<4; ii++)
    {
      dist[ii] = ((uint16_t)packet2[4+ii*4] + 
        (((uint16_t)(packet2[5+ii*4] & 0x3f))<<8))/1000.;
      invalid[ii] = (packet2[5] & 0x80)>>7;
      warning[ii] = (packet2[5] & 0x40)>>6;
      signal[ii] = (uint16_t)packet2[6+ii*4] + 
        (((uint16_t)(packet2[7+ii*4]))<<8);
    }
		// copy most recent avgspeed
		memcpy(&(pack->buffer[PACKET_HEADER_SIZE+(360*2)*sizeof(float)]), 
				&avgspeed, sizeof(float));
		// load into packet
    for (ii=0; ii<4; ii++)
    {
			ang = pos_offset+ii;
			// copy distance
      val = dist[ii];
      if (invalid[ii]!=0) val = 0.0;
			memcpy(&(pack->buffer[PACKET_HEADER_SIZE+(ang*2)*sizeof(float)]),
					&val, sizeof(float));
			
			// copy strength
			val = signal[ii];
			memcpy(&(pack->buffer[PACKET_HEADER_SIZE+(ang*2+1)*sizeof(float)]),
					&val, sizeof(float));
      count ++;
    }
  }
  return pos_offset;
}

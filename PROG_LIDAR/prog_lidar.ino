
uint8_t packet[22];
int packet_index;
float avgspeed;
int count, loc, lastloc;
char *buff, tag;
int packet_size, data_size;

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
  packet_size = 2048;
  data_size = 360*4;
  buff = new char [packet_size];
  tag = 'L';
	count = 0;
  memset(buff, 0x11, 3);
  memcpy(buff+3, &packet_size, sizeof(int));
  memcpy(buff+7, &data_size, sizeof(int));
  memcpy(buff+11, &tag, sizeof(char));
}

void loop ()
{
  byte in;
  
  while (Serial1.available())
  {
    in = Serial1.read();
    if (in==0xFA)
    {
      packet[0] = 0xFA;
      packet_index = 1;
    } else if (packet_index > 0)
    {
      packet[packet_index] = in;
      packet_index++;
      if (packet_index==22)
      {
        loc = parsePacket();
        packet_index = 0;
        if (count > 360)
        {
          // send buffer
          Serial.write(buff, sizeof(char)*packet_size);
          memset(buff+12, 0x00, sizeof(char)*data_size);
          count = 0;
        }
        lastloc = loc;
      }
    }
  }
}

int parsePacket()
{
  int ii;
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
    temp = (uint16_t)packet[ii*2] + (((uint16_t)packet[ii*2+1])<<8);
    chk32 = (chk32 << 1) + temp;
  }
  chk32 = (chk32 & 0x7fff) + ((chk32 >> 15) & 0x7fff);
  chk_expected = packet[20] | ((packet[21])<<8);
  
  // proceed if checksum is valid
  if (chk32 == chk_expected)
  {
    pos_offset = (packet[1]-0xA0) * 4;
    speed = ((uint16_t)packet[2] + (((uint16_t)packet[3])<<8))/64.;
    
    for (ii=0; ii<4; ii++)
    {
      dist[ii] = ((uint16_t)packet[4+ii*4] + 
        (((uint16_t)(packet[5+ii*4] & 0x3f))<<8))/1000.;
      invalid[ii] = (packet[5] & 0x80)>>7;
      warning[ii] = (packet[5] & 0x40)>>6;
      signal[ii] = (uint16_t)packet[6+ii*4] + 
        (((uint16_t)(packet[7+ii*4]))<<8);
    }

      for (ii=0; ii<4; ii++)
      {
        val = dist[ii];
        if (invalid[ii]!=0) val = 0.0;
        memcpy(buff+12+(pos_offset+ii)*sizeof(float),
          &val, sizeof(float));
        
        count ++;
      }
  }
  return pos_offset;
}

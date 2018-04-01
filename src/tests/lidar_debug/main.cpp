#include <iostream>
#include <string>
#include <stdio.h>
#include <thread>
#include <cmath>
#include "/home/ubuntu/PROJECTS/HEX/LIB_SERIAL/serial.h"

using namespace std;


int main(void)
{
	int ii, ix, iy, size;
	int cx, cy;
	char *buffer;
	serial ser;
	packet *recv;
	float dist, strength, speed;
	int *map;

	cx = -5;
	cy = 5;
	ser.init("/dev/ttymxc3");

	buffer = new char [1024];
/*
	for (ii=0; ii<3; ii++)
	{
		cout << "sending.." << endl;
		ser.send(buffer, 1024);
		sleep(1);
	}
	cout << "done" << endl;
*/
	size = 32;
	map = new int [size*size];
	while (true)
	{
		// get a packet
		recv = ser.recv('L', true); // blocking = true
		memset(map, 0x00, size*size*sizeof(int));
		for (ii=0; ii<360; ii++)
		{
			memcpy(&dist, &(recv->buffer[PACKET_HEADER_SIZE+ii*2*sizeof(float)]), 
						sizeof(float));
			memcpy(&strength, &(recv->buffer[PACKET_HEADER_SIZE+(ii*2+1)*sizeof(float)]), 
						sizeof(float));
			ix = size/2 + dist*5*cos(ii*0.01745)+cx;
			iy = size/2 - dist*5*sin(ii*0.01745)+cy;
			if (ix >= 0 && ix < size && iy >= 0 && iy < size)
				map[iy*size+ix] = max(min((int)(log(strength)*3.0/log(10.0)), 9), 0);
		}
		memcpy(&speed, &(recv->buffer[PACKET_HEADER_SIZE+360*2*sizeof(float)]),
				sizeof(float));
		for (iy=0; iy<size; iy++)
		{
			for (ix=0; ix<size; ix++)
			{
				if (ix==size/2+cx && iy==size/2+cy) cout << "O ";
				else if (map[iy*size+ix]) cout << map[iy*size+ix] << " ";
				else cout << ". ";
			}
			cout << endl;
		}
		cout << speed << endl;
	}



	while (ser.recv_queue_access) {}
	ser.recv_queue_access = true;
	cout << "recv queue: " << ser.recv_queue_packets.size() << endl;
	ser.recv_queue_access = false;

	ser.close();

}

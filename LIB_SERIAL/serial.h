#ifndef SERIAL_H
#define SERIAL_H

#include <iostream>
#include <vector>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <sys/time.h>
#include <ctime>
#include <thread>
#include <mutex>
#include "/home/bgreer/PROJECTS/HEX/LIB_PACKET/packet.h"

#define INPUT_BUFFER_SIZE 16384

using namespace std;


class serial
{
public:
	bool initialized, listening, debug;
	int fd;
	thread listener;
	// send queue
	mutex send_queue_mutex;
	vector<packet*> send_queue_packets;
	vector<bool*> send_queue_confirm;
	// recv queue
	mutex recv_queue_mutex;
	vector<packet*> recv_queue_packets;

	serial();
	void init(const char *portname, bool debugflag = false);
	void init_old(const char *portname, bool debugflag = false);
	void close();
	void send(packet *pack, bool blocking = false);
	packet* recv(unsigned char tag, uint8_t tag2, bool blocking);
	int set_interface_attribs (int speed, int parity);
	void set_blocking (int should_block);
};


void listener_loop (serial *ser);

#endif

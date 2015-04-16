#include "serial.h"

void listener_loop (serial *ser)
{
	int ii, size, in, start;
	int psize, dsize, ind;
	bool *ptr, loading;
	unsigned char *buff, *input, tag;
	float lasttime, currtime;
	struct timeval tv;
	size_t ret;
	packet *currpacket;

	start = 0;
	loading = false;
	input = new unsigned char [INPUT_BUFFER_SIZE];
	gettimeofday(&tv, NULL);
	currtime = (tv.tv_sec-1422700000) + tv.tv_usec*1e-6;
	lasttime = currtime;


	while (ser->listening)
	{
		ser->recv_queue_mutex.lock();
		// delete some if we have too many
		if (ser->recv_queue_packets.size() > 64)
		{
			cout << "WARNING: too many unclaimed packets!" << endl;
			for (ii=0; ii<ser->recv_queue_packets.size()-64; ii++)
			{
				delete ser->recv_queue_packets[ii];
				ser->recv_queue_packets.erase(ser->recv_queue_packets.begin());
			}
		}
		ser->recv_queue_mutex.unlock();
		// check for things to send
		if (ser->send_queue_packets.size() > 0)
		{
			ser->send_queue_mutex.lock();
			size = *(ser->send_queue_packets[0]->p_packet_size);
			buff = ser->send_queue_packets[0]->buffer;
			ptr = ser->send_queue_confirm[0];
			ser->send_queue_packets.erase(ser->send_queue_packets.begin());
			ser->send_queue_confirm.erase(ser->send_queue_confirm.begin());
			ret = write(ser->fd, buff, size);
			ser->send_queue_mutex.unlock();
			if (ptr != NULL) *ptr = true;
		}

		// only check input if we've waited a bit
		gettimeofday(&tv, NULL);
		currtime = (tv.tv_sec-1422700000) + tv.tv_usec*1e-6;
		if (currtime - lasttime > 0.05)
		{
			// check for things to recv
			in = read(ser->fd, input, INPUT_BUFFER_SIZE);
			for (ii=0; ii<in; ii++)
			{
				if (ser->debug) cout << input[ii] << flush;
				// look for packet start signal
				if (input[ii]==0x11) start++;
				else start = max(0,start-1);
				if (start==3)
				{
					if (loading) // we were already loading a packet
					{
						cout << "WARNING: packet collision" << endl;
						delete currpacket;
						loading = false;
					}
					// grab rest of header
					if (ii+PACKET_HEADER_SIZE-3 < in)
					{
						memcpy(&psize, input+ii+PACKET_HEADER_PACKET_SIZE-2, sizeof(int));
						memcpy(&dsize, input+ii+PACKET_HEADER_DATA_SIZE-2, sizeof(int));
						memcpy(&tag, input+ii+PACKET_HEADER_TAG-2, sizeof(unsigned char));
						ii += PACKET_HEADER_SIZE-3;
						currpacket = new packet(dsize, tag, psize);
						loading = true;
						ind = 0;
						start = 0;
					} else {
						cout << "WARNING: packet miss" << endl; // TODO prevent this
					}
				}
				if (loading) { // load data into packet
					currpacket->buffer[ind+PACKET_HEADER_SIZE-1] = input[ii]; // TODO: change to memcpy
					ind ++;
					if (ind == dsize+1) // done loading
					{
						loading = false;
						// add to recv queue
						ser->recv_queue_mutex.lock();
						ser->recv_queue_packets.push_back(currpacket);
						ser->recv_queue_mutex.unlock();
					}
				}
			}
			lasttime = currtime;
		} else {
			usleep(100);
		}
	}

}


serial::serial ()
{
	initialized = false;
}


void serial::init (const char *portname, bool debugflag)
{
	int r;
	struct termios options;
	struct serial_struct kernel_serial_settings;

	fd = open(portname, O_RDWR);
	if (fd < 0)
	{
		printf("HARD ERROR: %d opening %s: %s", errno, portname, strerror (errno));
		exit(-1);
	}
	if (tcgetattr(fd, &options) < 0) printf("unable to get serial parms\n");
	cfmakeraw(&options);
	if (cfsetspeed(&options, 115200) < 0) printf("error in cfsetspeed\n");
	if (tcsetattr(fd, TCSANOW, &options) < 0) printf("unable to set baud rate\n");
	r = ioctl(fd, TIOCGSERIAL, &kernel_serial_settings);
	if (r >= 0)
	{
		kernel_serial_settings.flags |= ASYNC_LOW_LATENCY;
		r = ioctl(fd, TIOCSSERIAL, &kernel_serial_settings);
		if (r >= 0) printf("set linux low latency mode\n");
	}
	debug = debugflag;
	listening = true;
	listener = thread(listener_loop, this);
	initialized = true;
}

void serial::init_old (const char *portname, bool debugflag)
{
	fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
		printf("HARD ERROR: %d opening %s: %s", errno, portname, strerror (errno));
		exit(-1);
	}
//	set_interface_attribs (B230400, 0);
	set_interface_attribs (B115200, 0);
	set_blocking (0);

	debug = debugflag;
	listening = true;
	listener = thread(listener_loop, this);
	initialized = true;
}

void serial::close ()
{
	listening = false;
	listener.join();
}

void serial::send (packet *pack, bool blocking)
{
	bool *sent;
	bool sent_local;

	sent = NULL;
	if (blocking)
	{
		sent = new bool;
		*sent = false;
	}
	send_queue_mutex.lock();
	pack->setChecksum();
	send_queue_packets.push_back(pack);
	send_queue_confirm.push_back(sent);
	send_queue_mutex.unlock();

	if (blocking)
	{
		sent_local = false;
		while (!sent_local)
		{
			usleep(100);
			send_queue_mutex.lock();
			sent_local = sent;
			send_queue_mutex.unlock();
		}
		delete sent;
	}
}

packet* serial::recv (unsigned char tag, bool blocking)
{
	int ii, ind;
	bool found = false;
	packet *p = NULL;

	do
	{
		recv_queue_mutex.lock();
		// look for oldest packet with matching tag
		for (ii=0; ii<recv_queue_packets.size(); ii++)
		{
			if (recv_queue_packets[ii]->getTag() == tag)
			{
				ind = ii;
				found = true;
				p = recv_queue_packets[ii];
				recv_queue_packets.erase(recv_queue_packets.begin()+ii);
				// jump to end of loop
				ii = recv_queue_packets.size();
			}
		}
		recv_queue_mutex.unlock();
		if (!found && blocking) usleep(1000);
	} while (!found && blocking);
	return p;
}

	int serial::set_interface_attribs (int speed, int parity)
	{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // ignore break signal
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 0;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
	}

	void serial::set_blocking (int should_block)
	{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 0;            	// 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf ("error %d setting term attributes", errno);
	}




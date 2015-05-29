#include "logger.h"

void logger_loop (logger *log)
{
	int ii, numwritten;
	double lasttime, currtime;
	struct timeval tv;
	data_chunk *p;

	gettimeofday(&tv, NULL);
	currtime = (tv.tv_sec-1431710000) + tv.tv_usec*1e-6;
	lasttime = currtime;

	while (log->listening)
	{
		numwritten = 0;
		// check for things to send, only send 10 max before unlocking mutex
		log->queue_mutex.lock();

		// if we have lost data, log a warning
		if (log->lostdata)
		{
			if (log->plaintext)
				log->file << "WARNING: logging queue filled!" << endl;
			else
				log->file.write("QUEUE FULL", 10);
			log->lostdata = false;
		}
		while (log->queue_num > 0 && numwritten < 10)
		{
			// grab the pointer from the queue
			p = log->parcel[log->queue_start];
			// write chunk
			if (log->plaintext)
			{
				// use ascii text
				log->file << p->time << "\t" << p->tag << "\t" << p->num;
				for (ii=0; ii<p->num; ii++)
					log->file << "\t" << p->data[ii];
				log->file << endl;
			} else {
				// use binary
				log->file.write(reinterpret_cast<char*>(&(p->time)), sizeof(double));
				log->file.write(reinterpret_cast<char*>(&(p->tag)), sizeof(unsigned char));
				log->file.write(reinterpret_cast<char*>(&(p->num)), sizeof(int));
				log->file.write(reinterpret_cast<char*>(p->data), sizeof(float)*p->num);
			}
			// sort out new queue status
			log->parcel[log->queue_start] = NULL;
			log->queue_start = (log->queue_start+1) % QUEUE_SIZE;
			log->queue_num --;
			// free memory
			delete p;
			numwritten ++;
		}
		log->queue_mutex.unlock();
	}
}


logger::logger ()
{
	initialized = false;
}


void logger::init (const char *filename, bool textflag)
{

	file.open(filename);
	plaintext = textflag;
	queue_start = 0;
	queue_end = 0;
	queue_num = 0;
	lostdata = false;
	listening = true;
	listener = thread(logger_loop, this);
	initialized = true;
}


void logger::close ()
{
	bool empty = false;
	// wait for queue to empty
	queue_mutex.lock();
	empty = (queue_start == queue_end);
	queue_mutex.unlock();

	cout << "waiting for logging to finish.." << endl;
	if (!empty)
	{
//		usleep(10000);
		queue_mutex.lock();
		empty = (queue_start == queue_end);
		queue_mutex.unlock();
	}
	cout << "queue empty, closing file" << endl;

	listening = false;
	listener.join();
	file.close();
}

// returns false if the send op failed
// this is likely due to the log queue being full
bool logger::send (data_chunk *d)
{
	bool ret;

	queue_mutex.lock();
	// do we have enough space?
	if (queue_num < QUEUE_SIZE)
	{
		parcel[queue_end] = d;
		queue_end = (queue_end+1)%QUEUE_SIZE;
		queue_num ++;
		ret = true;
	} else {
		lostdata = true;
		ret = false;
	}
	queue_mutex.unlock();
	return ret;
}



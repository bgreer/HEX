#include "header.h"

#ifndef gettime
#define gettime
double getTime()
{
	double ret;
	struct timeval tv;
	gettimeofday(&tv, NULL);
	ret = (tv.tv_sec) + tv.tv_usec*1e-6;
	return ret;
}
#endif

void setLEDInput (uint8_t id)
{
	int fd;
	char buf[128];
	ssize_t ret;

	sprintf(buf, "/sys/class/gpio/gpio%d/direction", id);
	fd = open(buf, O_WRONLY);
	ret = write(fd, "out", 3); 
	close(fd);
}

void setLED (uint8_t id, bool value)
{
	int fd;
	char buf[128];
	ssize_t ret;

	sprintf(buf, "/sys/class/gpio/gpio%d/value", id);
	fd = open(buf, O_WRONLY);
	if (value) ret = write(fd, "1", 1); 
	else ret = write(fd, "0", 1);
	close(fd);
}

bool getButtonPress (uint8_t id, bool blocking)
{
	int fd;
	char buf[128], val;
	ssize_t ret;

	sprintf(buf, "/sys/class/gpio/gpio%d/value", id);
	fd = open(buf, O_RDONLY);
	ret = read(fd, &val, 1);
	close(fd);
	usleep(50000);
	while (val!='0' && blocking)
	{
		usleep(100000);
		fd = open(buf, O_RDONLY);
		ret = read(fd, &val, 1);
		close(fd);
	}
	close(fd);
	return (val=='0');
}

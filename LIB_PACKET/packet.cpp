//#include "/home/ubuntu/PROJECTS/HEX/LIB_PACKET/packet.h"
#include "packet.h"

packet::packet (int datasize, unsigned char tag, int nearest)
{
	if (nearest == 0)
	{
		data_size = datasize;
		packet_size = data_size+PACKET_HEADER_SIZE;
		padding = 0;
	} else {
		data_size = datasize;
		packet_size = (int)((data_size+PACKET_HEADER_SIZE)/nearest + 1)*nearest;
		padding = packet_size - (data_size+PACKET_HEADER_SIZE);
	}
	buffer = (unsigned char*) malloc(packet_size*sizeof(unsigned char));
//	buffer = new unsigned char [packet_size];
//	buffer = new char [packet_size];
	memset(buffer, 0x00, sizeof(unsigned char) * packet_size);
	memset(buffer, 0x11, 3 * sizeof(unsigned char));
	memcpy(buffer+3, &packet_size, sizeof(int));
	memcpy(buffer+7, &data_size, sizeof(int));
	buffer[PACKET_HEADER_TAG] = tag; // tag
	buffer[PACKET_HEADER_CHECKSUM] = 0x00; // checksum
}

unsigned char packet::getTag ()
{
	return buffer[PACKET_HEADER_TAG];
}

void packet::setTag (unsigned char newtag)
{
	buffer[PACKET_HEADER_TAG] = newtag;
}

bool packet::verify()
{
	return (buffer[PACKET_HEADER_CHECKSUM] == computeChecksum());
}

unsigned char packet::computeChecksum ()
{
	int ii;
	unsigned char checksum = 0x00;
	for (ii=0; ii<data_size; ii++)
		checksum ^= buffer[ii+PACKET_HEADER_SIZE];
	return checksum;
}

void packet::setChecksum ()
{
	buffer[PACKET_HEADER_CHECKSUM] = computeChecksum();
}

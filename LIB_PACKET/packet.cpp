//#include "/home/ubuntu/PROJECTS/HEX/LIB_PACKET/packet.h"
#include "packet.h"

packet::packet (int datasize, unsigned char tag, int nearest, 
		unsigned char* prebuff)
{
	int dsize, psize;

	// error catching?
	if (datasize < 0) return;

	dsize = datasize;
	if (nearest == 0)
		psize = dsize+PACKET_HEADER_SIZE+PACKET_FOOTER_SIZE;
	else
		psize = (int)((dsize+PACKET_HEADER_SIZE+PACKET_FOOTER_SIZE)/nearest + 1)*nearest;

	// allocate space for everything
	if (prebuff == NULL)
	{
		usingprebuff = false;
		buffer = (unsigned char*) malloc(psize*sizeof(unsigned char));
	} else {
		usingprebuff = true;
		buffer = prebuff;
	}
	// clear memory, just in case
	memset(buffer, 0x00, sizeof(unsigned char) * psize);
	// set first 3 bytes as 0x11, used to identify incoming packets
	memset(buffer, 0x11, 3 * sizeof(unsigned char));
	// set last byte for easier sending
	buffer[psize-1] = '\n';

	// set pointers to important parts in memory
	data = &(buffer[13]);
	p_packet_size = (int*)(&(buffer[3]));
	p_data_size = (int*)(&(buffer[7]));
	p_tag = &(buffer[11]);
	p_checksum = &(buffer[12]);
	// set values
	*p_packet_size = psize;
	*p_data_size = dsize;
	*p_tag = tag;

}

unsigned char packet::getTag ()
{
	return *p_tag;
}

void packet::setTag (unsigned char newtag)
{
	*p_tag = newtag;
}

bool packet::verify()
{
	return (*p_checksum == computeChecksum());
}

unsigned char packet::computeChecksum ()
{
	int ii;
	unsigned char checksum = 0x00;
	for (ii=0; ii<*p_data_size; ii++)
		checksum ^= data[ii];
	return checksum;
}

void packet::setChecksum ()
{
	*p_checksum = computeChecksum();
}

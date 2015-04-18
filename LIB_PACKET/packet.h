#include <stdlib.h>
#include <iostream>
#include <string.h>

using namespace std;

// header:
// [0,2] = 0x11, 0x11, 0x11 (packet start)
// [3,6] = (int) packet_size
// [7,10] = (int) data_size
// [11] = (char) tag
// [12] = checksum
#define PACKET_HEADER_SIZE 13
#define PACKET_FOOTER_SIZE 1
#define PACKET_HEADER_PACKET_SIZE 3
#define PACKET_HEADER_DATA_SIZE 7
#define PACKET_HEADER_CHECKSUM 12
#define PACKET_HEADER_TAG 11
// an object that can be sent back and forth between the cpu and arduino
// has a few helper functions for wrapping, checksums, etc

class packet
{
public:
	unsigned char *buffer; // stores everything
	// pointers to sections of the main buffer:
	int *p_packet_size, *p_data_size;
	unsigned char *p_checksum, *p_tag, *data;
	bool usingprebuff;

	// CONSTRUCTOR
	packet (int datasize, unsigned char tag, int nearest=0, 
			unsigned char* prebuff = NULL);

	~packet ()
	{
		if (!usingprebuff)
			free(buffer);
	}
	unsigned char getTag ();
	void setTag (unsigned char newtag);
	bool verify();
	unsigned char computeChecksum ();
	void setChecksum ();

	
};

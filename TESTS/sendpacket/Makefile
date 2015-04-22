
BASEDIR = /home/bgreer/PROJECTS/HEX
SERIALDIR = $(BASEDIR)/LIB_SERIAL
PACKETDIR = $(BASEDIR)/LIB_PACKET
HEXAPODDIR = $(BASEDIR)/LIB_HEXAPOD

SERIAL = $(SERIALDIR)/serial.cpp $(SERIALDIR)/serial.h
PACKET = $(PACKETDIR)/packet.cpp $(PACKETDIR)/packet.h
HEXAPOD = $(HEXAPODDIR)/hexapod.cpp $(HEXAPODDIR)/hexapod.h

sendpacket : main.cpp $(SERIAL) $(PACKET) $(HEXAPOD)
	g++ -std=c++0x -pthread -O2 -o sendpacket main.cpp $(SERIAL) $(PACKET) $(HEXAPOD) -lm

repeater : repeater.cpp $(SERIAL) $(PACKET) $(HEXAPOD)
	g++ -std=c++0x -pthread -O2 -o repeater repeater.cpp $(SERIAL) $(PACKET) $(HEXAPOD) -lm

clean :
	rm -f sendpacket repeater

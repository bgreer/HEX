
BASEDIR = /home/ubuntu/PROJECTS/HEX
SERIALDIR = $(BASEDIR)/LIB_SERIAL
PACKETDIR = $(BASEDIR)/LIB_PACKET

SERIAL = $(SERIALDIR)/serial.cpp $(SERIALDIR)/serial.h
PACKET = $(PACKETDIR)/packet.cpp $(PACKETDIR)/packet.h

lidar_debug : main.cpp $(SERIAL) $(PACKET)
	g++ -std=c++0x -pthread -g -o lidar_debug main.cpp $(SERIAL) $(PACKET)

clean :
	rm -f lidar_debug

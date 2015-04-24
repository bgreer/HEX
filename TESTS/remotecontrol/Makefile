
BASEDIR = /home/bgreer/PROJECTS/HEX
SERIALDIR = $(BASEDIR)/LIB_SERIAL
PACKETDIR = $(BASEDIR)/LIB_PACKET
HEXAPODDIR = $(BASEDIR)/LIB_HEXAPOD

SERIAL = $(SERIALDIR)/serial.cpp $(SERIALDIR)/serial.h
PACKET = $(PACKETDIR)/packet.cpp $(PACKETDIR)/packet.h
HEXAPOD = $(HEXAPODDIR)/hexapod.cpp $(HEXAPODDIR)/hexapod.h
SDL = -I/usr/local/include/SDL -L/usr/local/lib -lSDL

LIBS = $(SDL) -D_GNU_SOURCE=1 -D_REENTRANT -pthread -lpthread -lrt -ldl -lm

remote : main.cpp $(SERIAL) $(PACKET) $(HEXAPOD)
	g++ -std=c++0x -O3 -g -o remote main.cpp $(SERIAL) $(PACKET) $(HEXAPOD) $(LIBS)

clean :
	rm -f remote


BASEDIR = /home/bgreer/PROJECTS/HEX
SERIALDIR = $(BASEDIR)/LIB_SERIAL
PACKETDIR = $(BASEDIR)/LIB_PACKET
HEXAPODDIR = $(BASEDIR)/LIB_HEXAPOD
LOGGERDIR = $(BASEDIR)/LIB_LOGGER

SERIAL = $(SERIALDIR)/serial.cpp $(SERIALDIR)/serial.h
PACKET = $(PACKETDIR)/packet.cpp $(PACKETDIR)/packet.h
HEXAPOD = $(HEXAPODDIR)/hexapod.cpp $(HEXAPODDIR)/hexapod.h
LOGGER = $(LOGGERDIR)/logger.cpp $(LOGGERDIR)/logger.h
SDL = -I/usr/local/include/SDL -L/usr/local/lib -lSDL

LIBS = -lpthread -lrt -ldl -lm

logtest : main.cpp $(LOGGER)
	g++ -std=c++0x -pthread -O3 -g -o logtest main.cpp $(LOGGER)

clean :
	rm -f logtest

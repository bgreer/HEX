
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

LIBS = $(SDL) -D_GNU_SOURCE=1 -D_REENTRANT -pthread -lpthread -lrt -ldl -lm

# this needs to be cleaned up
hex : main.cpp manual.cpp actions.cpp monitor.cpp $(SERIAL) $(PACKET) $(HEXAPOD) $(LOGGER)
	g++ -std=c++0x -O3 -g -o hex main.cpp manual.cpp actions.cpp monitor.cpp $(SERIAL) $(PACKET) $(HEXAPOD) $(LOGGER) $(LIBS)

clean :
	rm -f hex

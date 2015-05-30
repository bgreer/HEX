
BASEDIR = /home/bgreer/PROJECTS/HEX
SERIALDIR = $(BASEDIR)/LIB_SERIAL
PACKETDIR = $(BASEDIR)/LIB_PACKET
HEXAPODDIR = $(BASEDIR)/LIB_HEXAPOD
SLAMDIR = $(BASEDIR)/LIB_SLAM
LOGGERDIR = $(BASEDIR)/LIB_LOGGER

SERIAL = $(SERIALDIR)/serial.cpp $(SERIALDIR)/serial.h
PACKET = $(PACKETDIR)/packet.cpp $(PACKETDIR)/packet.h
HEXAPOD = $(HEXAPODDIR)/hexapod.cpp $(HEXAPODDIR)/hexapod.h
SLAM = $(SLAMDIR)/slam.cpp $(SLAMDIR)/slam.h $(SLAMDIR)/scan.h
LOGGER = $(LOGGERDIR)/logger.cpp $(LOGGERDIR)/logger.h
SDL = -I/usr/local/include/SDL -L/usr/local/lib -lSDL

LIBS = $(SDL) -D_GNU_SOURCE=1 -D_REENTRANT -pthread -lpthread -lrt -ldl -lfftw3_threads -lfftw3 -lm

# this needs to be cleaned up
hex : main.cpp manual.cpp actions.cpp monitor.cpp $(SERIAL) $(PACKET) $(HEXAPOD) $(LOGGER) $(SLAM)
	g++ -std=c++0x -O3 -g -o hex main.cpp manual.cpp actions.cpp monitor.cpp $(SERIAL) $(PACKET) $(HEXAPOD) $(LOGGER) $(SLAM) $(LIBS)

clean :
	rm -f hex


BASEDIR = /home/bgreer/PROJECTS/HEX
SERIALDIR = $(BASEDIR)/LIB_SERIAL
PACKETDIR = $(BASEDIR)/LIB_PACKET
HEXAPODDIR = $(BASEDIR)/LIB_HEXAPOD
SLAMDIR = $(BASEDIR)/LIB_SLAM
LOGGERDIR = $(BASEDIR)/LIB_LOGGER
NAVDIR = $(BASEDIR)/LIB_AUTONAV

SERIAL = $(SERIALDIR)/serial.cpp $(SERIALDIR)/serial.h
PACKET = $(PACKETDIR)/packet.cpp $(PACKETDIR)/packet.h
HEXAPOD = $(HEXAPODDIR)/hexapod.cpp $(HEXAPODDIR)/hexapod.h
SLAM = $(SLAMDIR)/slam.cpp $(SLAMDIR)/slam.h $(SLAMDIR)/scan.h
LOGGER = $(LOGGERDIR)/logger.cpp $(LOGGERDIR)/logger.h
NAV = $(NAVDIR)/autonav.cpp $(NAVDIR)/autonav.h
SDL = -I/usr/local/include/SDL -L/usr/local/lib -lSDL

TCLAP = -I/home/bgreer/SOFTWARE/tclap-1.2.1/include
LIBS = $(SDL) -D_GNU_SOURCE=1 -D_REENTRANT -pthread -lpthread -lrt -ldl $(TCLAP) -lfftw3_threads -lfftw3 -lm

# this needs to be cleaned up
hex : main.cpp manual.cpp actions.cpp monitor.cpp serial.o packet.o hexapod.o logger.o slam.o autonav.o header.h
	g++ -std=c++0x -O3 -g -o hex main.cpp manual.cpp actions.cpp monitor.cpp serial.o packet.o hexapod.o logger.o slam.o autonav.o $(LIBS)

hexapod.o : $(HEXAPOD)
	g++ -std=c++0x -O3 -c $(HEXAPOD) $(LIBS)

packet.o : $(PACKET)
	g++ -std=c++0x -O3 -c $(PACKET) $(LIBS)

serial.o : $(SERIAL)
	g++ -std=c++0x -O3 -c $(SERIAL) $(LIBS)

slam.o : $(SLAM)
	g++ -std=c++0x -O3 -c $(SLAM) $(LIBS)

logger.o : $(LOGGER)
	g++ -std=c++0x -O3 -c $(LOGGER) $(LIBS)

autonav.o : $(NAV)
	g++ -std=c++0x -O3 -c $(NAV) $(LIBS)

clean :
	rm -f hex *.o

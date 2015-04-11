BASEDIR = /home/bgreer/PROJECTS/HEX
SERIALDIR = $(BASEDIR)/LIB_SERIAL
PACKETDIR = $(BASEDIR)/LIB_PACKET
HEXAPODDIR = $(BASEDIR)/LIB_HEXAPOD
SLAMDIR = $(BASEDIR)/LIB_SLAM

SERIAL = $(SERIALDIR)/serial.cpp $(SERIALDIR)/serial.h
PACKET = $(PACKETDIR)/packet.cpp $(PACKETDIR)/packet.h
HEXAPOD = $(HEXAPODDIR)/hexapod.cpp $(HEXAPODDIR)/hexapod.h
SLAM = $(SLAMDIR)/slam.cpp $(SLAMDIR)/slam.h $(SLAMDIR)/scan.h

LIBS = -I/usr/include/SDL -D_GNU_SOURCE=1 -D_REENTRANT -L/usr/lib64 -lSDL -pthread -lpthread -lrt -ldl -lfftw3 -lm

sdltest : main.cpp panel.h $(SLAM)
	g++ -std=c++0x -O3 -o sdltest main.cpp $(SLAM) $(LIBS)

clean : 
	rm -f *.o sdltest

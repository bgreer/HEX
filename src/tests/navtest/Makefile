
BASEDIR = /home/bgreer/PROJECTS/HEX
HEXAPODDIR = $(BASEDIR)/LIB_HEXAPOD
SLAMDIR = $(BASEDIR)/LIB_SLAM
NAVDIR = $(BASEDIR)/LIB_AUTONAV

HEXAPOD = $(HEXAPODDIR)/hexapod.cpp $(HEXAPODDIR)/hexapod.h
SLAM = $(SLAMDIR)/slam.cpp $(SLAMDIR)/slam.h $(SLAMDIR)/scan.h
NAV = $(NAVDIR)/autonav.cpp $(NAVDIR)/autonav.h

LIBS = -pthread -lfftw3 -lm

navtest : main.cpp $(HEXAPOD) $(SLAM) $(NAV)
	g++ -std=c++0x -O3 -g -o navtest main.cpp $(HEXAPOD) $(SLAM) $(NAV) $(LIBS)

clean :
	rm -f navtest

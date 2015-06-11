BASEDIR = /home/bgreer/PROJECTS/HEX
SLAMDIR = $(BASEDIR)/LIB_SLAM
LOGGERDIR = $(BASEDIR)/LIB_LOGGER

SLAM = $(SLAMDIR)/slam.cpp $(SLAMDIR)/slam.h $(SLAMDIR)/scan.h
LOGGER = $(LOGGERDIR)/logger.cpp $(LOGGERDIR)/logger.h
SDL = -I/usr/include/SDL -L/usr/lib64 -lSDL

LIBS = $(SDL) -D_GNU_SOURCE=1 -D_REENTRANT -pthread -lpthread -lrt -ldl -lfftw3_threads -lfftw3 -lm

recon : main.cpp $(SLAM) ${LOGGER}
	g++ -std=c++0x -O3 -o recon main.cpp $(SLAM) ${LOGGER} $(LIBS)

clean : 
	rm -f *.o recon

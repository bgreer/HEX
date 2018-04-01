BASEDIR = /home/bgreer/PROJECTS/HEX
SLAMDIR = $(BASEDIR)/LIB_SLAM
LOGGERDIR = $(BASEDIR)/LIB_LOGGER

SLAM = $(SLAMDIR)/slam.cpp $(SLAMDIR)/slam.h $(SLAMDIR)/scan.h
LOGGER = $(LOGGERDIR)/logger.cpp $(LOGGERDIR)/logger.h
SDL = -I/usr/include/SDL -L/usr/lib64 -lSDL -L/usr/local/lib -lSDL_ttf
#SDL_TTF = -I/usr/local/include/SDL -L/usr/local/lib -lSDL_ttf

LIBS = $(SDL) -D_GNU_SOURCE=1 -D_REENTRANT -pthread -lpthread -lrt -ldl -lfftw3_threads -lfftw3 -lm

IMG_savepng.o : IMG_savepng.cpp IMG_savepng.h
	g++ -std=c++0x -O3 -g -c IMG_savepng.cpp $(LIBS)

recon : main.cpp IMG_savepng.o $(SLAM) ${LOGGER}
	g++ -std=c++0x -O3 -g -o recon main.cpp IMG_savepng.o $(SLAM) ${LOGGER} -lpng $(LIBS)

clean : 
	rm -f *.o recon

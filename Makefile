
BASEDIR = /home/bgreer/PROJECTS/HEX
AXSERVODIR = $(BASEDIR)/LIB_AXSERVO
SERIALDIR = $(BASEDIR)/LIB_SERIAL
PACKETDIR = $(BASEDIR)/LIB_PACKET
HEXAPODDIR = $(BASEDIR)/LIB_HEXAPOD
SLAMDIR = $(BASEDIR)/LIB_SLAM
LOGGERDIR = $(BASEDIR)/LIB_LOGGER
NAVDIR = $(BASEDIR)/LIB_AUTONAV

### TOOLS
CPP = /usr/share/arduino/hardware/tools/avr/bin/avr-g++
OBJCOPY = /usr/share/arduino/hardware/tools/avr/bin/avr-objcopy
AVRDUDE = /usr/share/arduino/hardware/tools/avrdude


SERVODIR = $(BASEDIR)/servo_controller
ARBOTIX_FLAGS = -std=gnu++11 -c -g -Os -Wall -fno-exceptions -ffunction-sections -fdata-sections -mmcu=atmega644p -DF_CPU=16000000L -MMD -DUSB_VID=null -DUSB_PID=null -DARDUINO=105 -D__PROG_TYPES_COMPAT__
ARBOTIX_INCLUDE = -I/home/bgreer/sketchbook/hardware/arbotix/cores/arbotix -I/home/bgreer/sketchbook/hardware/arbotix/variants/standard -I/home/bgreer/sketchbook/libraries/Bioloid
ARBOTIX_COMPILE = $(CPP) $(ARBOTIX_FLAGS) $(ARBOTIX_INCLUDE)
ARBOTIX_LINK = $(CPP)

SERIAL = $(SERIALDIR)/serial.cpp $(SERIALDIR)/serial.h
PACKET = $(PACKETDIR)/packet.cpp $(PACKETDIR)/packet.h
HEXAPOD = $(HEXAPODDIR)/hexapod.cpp $(HEXAPODDIR)/hexapod.h
SLAM = $(SLAMDIR)/slam.cpp $(SLAMDIR)/slam.h $(SLAMDIR)/scan.h
LOGGER = $(LOGGERDIR)/logger.cpp $(LOGGERDIR)/logger.h
NAV = $(NAVDIR)/autonav.cpp $(NAVDIR)/autonav.h
SDL = -I/usr/local/include/SDL -L/usr/local/lib -lSDL

TCLAP = -I/home/bgreer/SOFTWARE/tclap-1.2.1/include
LIBS = $(SDL) -D_GNU_SOURCE=1 -D_REENTRANT -pthread -lpthread -lrt -ldl $(TCLAP) -lfftw3_threads -lfftw3 -lm

arbotix_core :
	$(ARBOTIX_COMPILE) /home/bgreer/sketchbook/hardware/arbotix/cores/arbotix/*.c*
	/usr/share/arduino/hardware/tools/avr/bin/avr-ar rcs core.a *.o
	rm *.o *.d

bioloid :
	$(ARBOTIX_COMPILE) /home/bgreer/sketchbook/libraries/Bioloid/*.c*

packet :
	$(ARBOTIX_COMPILE) $(PACKETDIR)/*.c*

axservo : $(AXSERVODIR)/*.cpp $(AXSERVODIR)/*.h
	$(ARBOTIX_COMPILE) $(AXSERVODIR)/*.c*

scontrol : $(SERVODIR)/*.cpp arbotix_core bioloid packet axservo
	$(ARBOTIX_COMPILE) $(SERVODIR)/*.cpp 
	$(CPP) -Os -Wl,--gc-sections -mmcu=atmega644p -o scontrol.elf servo_controller.o core.a -lm
	$(objcopy) -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 scontrol.elf scontrol.eep 
	$(objcopy) -O ihex -R .eeprom scontrol.elf scontrol.hex
	rm *.d *.o

upload : scontrol
	/usr/share/arduino/hardware/tools/avrdude -C/usr/share/arduino/hardware/tools/avrdude.conf -v -v -v -v -patmega644p -carduino -P/dev/ttyUSB0 -b38400 -D -Uflash:w:scontrol.hex:i

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
	rm -f hex build/*.o *.d *.o

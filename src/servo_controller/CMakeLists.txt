cmake_minimum_required(VERSION 2.8)
project(HEX)

###############################################################################
## file globbing ##############################################################
###############################################################################


include_directories(include)
# grab sources
file(GLOB sources src/*.cpp)

add_executable(servo_controller ${sources})

#CPP = /usr/share/arduino/hardware/tools/avr/bin/avr-g++
#OBJCOPY = /usr/share/arduino/hardware/tools/avr/bin/avr-objcopy
#AVRDUDE = /usr/share/arduino/hardware/tools/avrdude
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR avr)

# remove rdynamic flag
SET(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS)
set(CMAKE_CXX_COMPILER avr-g++)

# Just for example add some compiler flags.
target_compile_options(servo_controller PUBLIC -std=gnu++11 -c -g -Os -Wall -fno-exceptions -ffunction-sections -fdata-sections -mmcu=atmega644p -DF_CPU=16000000L -MMD -DUSB_VID=null -DUSB_PID=null -DARDUINO=105 -D__PROG_TYPES_COMPAT_)

# This allows to include files relative to the root of the src directory with a <> pair
#ARBOTIX_INCLUDE = -I/home/bgreer/sketchbook/hardware/arbotix/cores/arbotix -I/home/bgreer/sketchbook/hardware/arbotix/variants/standard -I/home/bgreer/sketchbook/libraries/Bioloid
target_include_directories(servo_controller PUBLIC ../../libs/arbotix/Bioloid ../../hardware/arbotix/cores/arbotix ../../hardware/arbotix/variants/arbotix2/ ../../libs/axservo/include ../../libs/packet/include)

# This copies all resource files in the build directory.
# We need this, because we want to work with paths relative to the executable.
#file(COPY ${data} DESTINATION resources)

###############################################################################
## dependencies ###############################################################
###############################################################################

# This defines the variables Boost_LIBRARIES that containts all library names
# that we need to link into the program.
#find_package(Boost 1.36.0 COMPONENTS filesystem system REQUIRED)

target_link_libraries(servo_controller PUBLIC
  #${Boost_LIBRARIES}
  # here you can add any library dependencies
)




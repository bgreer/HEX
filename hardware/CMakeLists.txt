cmake_minimum_required(VERSION 2.8)
project(HEX)

set(CMAKE_BUILD_TYPE Release)


include_directories(arbotix/cores/arbotix)
file(GLOB core_sources arbotix/cores/arbotix/*.c*)

SET(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS)
SET(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS)
set(CMAKE_CXX_COMPILER avr-g++)
set(CMAKE_C_COMPILER avr-gcc)
set(CMAKE_AR avr-ar)
SET(CMAKE_CXX_ARCHIVE_CREATE "<CMAKE_AR> rcs <TARGET> <LINK_FLAGS> <OBJECTS>")
SET(CMAKE_C_ARCHIVE_CREATE "<CMAKE_AR> rcs <TARGET> <LINK_FLAGS> <OBJECTS>")

add_library(core STATIC ${core_sources})


target_compile_options(core PUBLIC -std=gnu++11 -c -g -Os -Wall -fno-exceptions -ffunction-sections -fdata-sections -mmcu=atmega644p -DF_CPU=16000000L -MMD -DUSB_VID=null -DUSB_PID=null -DARDUINO=105 -D__PROG_TYPES_COMPAT_)

target_include_directories(core PUBLIC arbotix/variants/arbotix2/)

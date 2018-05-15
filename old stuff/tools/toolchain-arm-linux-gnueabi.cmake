# CMake toolchain file for building ARM software on OI environment

# this one is important
SET(CMAKE_SYSTEM_NAME Linux)
#this one not so much
SET(CMAKE_SYSTEM_VERSION 1)

# specify the cross compiler
SET(CMAKE_C_COMPILER   /usr/bin/arm-linux-gnueabihf-gcc)
SET(CMAKE_CXX_COMPILER /usr/bin/arm-linux-gnueabihf-g++)
SET(CMAKE_STRIP /usr/bin/arm-linux-gnueabihf-strip)

# where is the target environment 
SET(CMAKE_FIND_ROOT_PATH /home/nurc/Code/nurobotics_ws /usr/arm-linux-gnueabihf)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
# set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY) 

set(Boost_INCLUDE_DIR /usr/arm-linux-gnueabihf/include)
set(Boost_LIBRARY_DIR /usr/arm-linux-gnueabihf/lib)

set(LIBUSB_1_INCLUDE_DIRS /usr/arm-linux-gnueabihf/include)
set(LIBUSB_1_LIBRARY /usr/arm-linux-gnueabihf/lib/libusb-1.0.a)

set(LIBPIXYUSB_0_INCLUDE_DIR /usr/arm-linux-gnueabihf/include)
set(LIBPIXYUSB_0_LIBRARY /usr/arm-linux-gnueabihf/lib/libpixyusb.a)

# set(BOOST_ROOT /home/nurc/Code/nurobotics_ws/boost)
# set(BOOST_INCLUDEDIR /home/nurc/Code/nurobotics_ws/boost/include)
# set(Boost_INCLUDE_DIR /home/nurc/Code/nurobotics_ws/boost/include)
# set(BOOST_LIBRARYDIR /home/nurc/Code/nurobotics_ws/boost/lib)
# set(Boost_LIBRARY_DIR /home/nurc/Code/nurobotics_ws/boost/lib)
# 
# set(LIBUSB_1_ROOT /home/nurc/Code/nurobotics_ws/libusb-1.0.9/install)
# set(LIBUSB_1_INCLUDE_DIR /home/nurc/Code/nurobotics_ws/libusb-1.0.9/install/include/libusb-1.0/)
# set(LIBUSB_1_INCLUDE_DIRS /home/nurc/Code/nurobotics_ws/libusb-1.0.9/install/include/libusb-1.0/)
# set(LIBUSB_1_LIBRARY_DIR /home/nurc/Code/nurobotics_ws/libusb-1.0.9/install/lib)
# set(LIBUSB_1_LIBRARY /home/nurc/Code/nurobotics_ws/libusb-1.0.9/install/lib/libusb-1.0.a)
# 
# set(LIBPIXYUSB_0_ROOT /home/nurc/Code/nurobotics_ws/pixy/cross_compilation)
# set(LIBPIXYUSB_0_INCLUDE_DIR /home/nurc/Code/nurobotics_ws/pixy/cross_compilation/include)
# set(LIBPIXYUSB_0_LIBRARY_DIR /home/nurc/Code/nurobotics_ws/pixy/cross_compilation/lib)
# set(LIBPIXYUSB_0_LIBRARY /home/nurc/Code/nurobotics_ws/pixy/cross_compilation/lib/libpixyusb.a)

cmake_minimum_required(VERSION 2.8)
project(nurc-lax-robot CXX)

# ARM GNUEABIHF CHANGES #
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_MODULE_DIR}/cmake")
set(CMAKE_EXE_LINKER_FLAGS "-static")

find_library(GLIBC_LIBRARY NAMES libc.a c)

add_library (GLIBC_LIBRARY STATIC IMPORTED)
set_target_properties (GLIBC_LIBRARY PROPERTIES
    IMPORTED_LOCATION /usr/arm-linux-gnueabihf/lib/libc.a)

add_library (PTHREAD_LIBRARY STATIC IMPORTED)
set_target_properties (PTHREAD_LIBRARY PROPERTIES
    IMPORTED_LOCATION /usr/arm-linux-gnueabihf/lib/libpthread.a)

#########################

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
set(ROBOTEQ_INCLUDE_DIR ../roboteq-controller/include)

find_package(Boost 1.49 COMPONENTS thread system chrono program_options REQUIRED)
find_package(libpixyusb REQUIRED)
find_package(libusb-1.0 REQUIRED)

add_library(roboteq
  ../roboteq-controller/src/roboteq_controller.cc
  ../roboteq-controller/src/myd_serial.cc)

add_library(laxrobot
  src/lax_robot.cc)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${LIBUSB_1_INCLUDE_DIRS}
  ${LIBPIXYUSB_0_INCLUDE_DIR}
  ${ROBOTEQ_INCLUDE_DIR})

file(STRINGS "cmake/VERSION" LIBPIXY_VERSION)
add_definitions(-D__LIBPIXY_VERSION__="${LIBPIXY_VERSION}")

add_executable(lax_robot_main src/lax_robot_main.cc)
# libpixyusb should always come before libboost and libusb #
target_link_libraries(lax_robot_main
  ${LIBPIXYUSB_0_LIBRARY}
  ${Boost_LIBRARIES}
  ${LIBUSB_1_LIBRARY}
  GLIBC_LIBRARY
  PTHREAD_LIBRARY
  roboteq
  laxrobot)

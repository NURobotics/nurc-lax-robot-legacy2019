cmake_minimum_required(VERSION 2.8)
project(nurc-lax-robot CXX)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
set(ROBOTEQ_INCLUDE_DIR ../roboteq-controller/include)
set(PIXY_LIBRARY_DIR ../pixy/buil/libpixyusb)

find_package(Boost 1.49 COMPONENTS thread system chrono REQUIRED)
find_package(libpixyusb REQUIRED)
find_package(libusb-1.0 REQUIRED)

add_library(libroboteq
  ../roboteq-controller/src/roboteq_controller.cc
  ../roboteq-controller/src/myd_serial.cc)

add_library(liblaxrobot
  src/lax_robot.cc)

include_directories(
  include
  ${ROBOTEQ_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  ${LIBUSB_1_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR})

add_executable(lax_robot_main src/lax_robot_main.cc)
target_link_libraries(lax_robot_main
  liblaxrobot
  libroboteq
  ${Boost_LIBRARIES}
  ${LIBUSB_1_LIBRARY})

cmake_minimum_required (VERSION 2.8)
project (lax_robot CXX)

set(ROBOTEQ_INCLUDE_DIR ../roboteq-controller/include)

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake" )

# set (CMAKE_FIND_LIBRARY_SUFFIXES ".a;.so")
# set (BUILD_SHARED_LIBRARIES OFF)
set (CMAKE_EXE_LINKER_FLAGS "-static")

find_library ( GLIBC_LIBRARY NAMES libc.a c )
find_library ( PTHREAD_LIBRARY NAMES libpthread.a )
find_package ( libpixyusb REQUIRED )
find_package ( libusb-1.0 REQUIRED )
find_package ( Boost 1.49 COMPONENTS COMPONENTS thread system chrono REQUIRED)

file(STRINGS "cmake/VERSION" LIBPIXY_VERSION)
add_definitions(-D__LIBPIXY_VERSION__="${LIBPIXY_VERSION}")

include_directories (src
                     include
                     ../../common
                     ${Boost_INCLUDE_DIR}
                     ${LIBUSB_1_INCLUDE_DIRS}
                     ${LIBPIXYUSB_0_INCLUDE_DIR}
                     ${ROBOTEQ_INCLUDE_DIR})

add_library (GLIBC_LIBRARY STATIC IMPORTED)
set_target_properties (GLIBC_LIBRARY PROPERTIES
    IMPORTED_LOCATION /usr/arm-linux-gnueabihf/lib/libc.a)

add_library (PTHREAD_LIBRARY STATIC IMPORTED)
set_target_properties (PTHREAD_LIBRARY PROPERTIES
    IMPORTED_LOCATION /usr/arm-linux-gnueabihf/lib/libpthread.a)

add_library(roboteq
  ../roboteq-controller/src/roboteq_controller.cc
  ../roboteq-controller/src/myd_serial.cc)

add_library(laxrobot
  src/lax_robot.cc)

# Add sources here... #
add_executable (lax_robot_main src/lax_robot_main.cc src/triangulation.cc)

# libpixyusb should always come before libboost and libusb #
# target_link_libraries (lax_robot_main roboteq)
# target_link_libraries (lax_robot_main laxrobot)
# target_link_libraries (lax_robot_main ${LIBPIXYUSB_0_LIBRARY})
# target_link_libraries (lax_robot_main pthread)
# target_link_libraries (lax_robot_main ${Boost_LIBRARIES})
# target_link_libraries (lax_robot_main ${LIBUSB_1_LIBRARY})
# target_link_libraries (lax_robot_main GLIBC_LIBRARY)
# target_link_libraries (lax_robot_main PTHREAD_LIBRARY)

# target_link_libraries (${LIBUSB_1_LIBRARY}
#   GLIBC_LIBRARY
#   PTHREAD_LIBRARY)

# target_link_libraries (PTHREAD_LIBRARY GLIBC_LIBRARY)
# target_link_libraries (pthread GLIBC_LIBRARY)

target_link_libraries (laxrobot
  roboteq
  ${LIBPIXYUSB_0_LIBRARY}
  ${Boost_LIBRARIES}
  ${LIBUSB_1_LIBRARY}
  PTHREAD_LIBRARY
  GLIBC_LIBRARY)

# target_link_libraries (${LIBPIXYUSB_0_LIBRARY}
#   ${Boost_LIBRARIES}
#   ${LIBUSB_1_LIBRARY}
#   PTHREAD_LIBRARY)

target_link_libraries (lax_robot_main
  roboteq
  laxrobot)

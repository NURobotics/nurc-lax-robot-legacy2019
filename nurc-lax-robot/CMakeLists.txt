cmake_minimum_required (VERSION 2.8)
project (nurc-lax-robot CXX)

set(ROBOTEQ_INCLUDE_DIR ....finish this)
set(PIXY_LIBRARIES ...finish this)

add_library(libroboteq ...finish this)

find_package (Boost 1.49 COMPONENTS thread system chrono REQUIRED)

include_dicrectories (include ${ROBOTEQ_INCLUDE_DIR} ${Boost_INCLUDE_DIR})

add_executable(lax_robot src/lax_robot.cc)
target_link_libraries(lax_robot libroboteq ${Boost_LIBRARIES} ${PIXY_LIBRARIES})
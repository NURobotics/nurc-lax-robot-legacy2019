set (CMAKE_CXX_STANDARD 11)

cmake_minimum_required (VERSION 2.8)

project (nurc_lax)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(cv_nurc_lax main.cpp)

target_link_libraries(cv_nurc_lax ${OpenCV_LIBS})

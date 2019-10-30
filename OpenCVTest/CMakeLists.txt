set (CMAKE_CXX_STANDARD 11)

cmake_minimum_required (VERSION 2.8)

project (opencv_test)

find_package(OpenCV REQUIRED)

find_package(CUDA  REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

include_directories("${CUDA_INCLUDE_DIRS}")

add_executable(opencv_test OpenCVTest.cpp)

target_link_libraries(opencv_test ${OpenCV_LIBS})


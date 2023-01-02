cmake_minimum_required(VERSION 3.13)  # CMake version check
project(test_roboteq)               # Create project 
set(CMAKE_CXX_STANDARD 14)            # Enable c++14 standard

set(CMAKE_BINARY_DIR bin)

# Add main.cpp file of project root directory as source file
set(SOURCE_FILES 
	src/main.cpp
	src/RoboteqDevice.cpp	
)

# Add executable target with source files listed in SOURCE_FILES variable
add_executable(test_roboteq ${SOURCE_FILES})
set (CMAKE_CXX_STANDARD 11)

cmake_minimum_required (VERSION 2.8)

project (nurc_lax)

include_directories("C:/opencv/build/include")


add_executable(cv_nurc_lax main.cpp)

target_link_libraries(cv_nurc_lax "C:/opencv/build/include")

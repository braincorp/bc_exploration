cmake_minimum_required(VERSION 2.8.3)
project(astar_cpp)
set(CMAKE_BUILD_TYPE RELEASE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -Wall -shared -fpic")
set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)

file(GLOB all_files "inc/exploration/*.h" "src/exploration/*.cpp")

add_library(exploration SHARED ${all_files})
set_target_properties(exploration PROPERTIES SUFFIX ".so" PREFIX "")
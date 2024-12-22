cmake_minimum_required(VERSION 3.10)
project(Error_state_kalman_filter)

set(CMAKE_CXX_STANDARD 17)

option(BUILD_DEMO "Build demo" ON)

include_directories(demo/EmbeddedMath)

if (BUILD_DEMO)
    add_executable(eskf_demo demo/demo.cpp)
endif (BUILD_DEMO)


cmake_minimum_required(VERSION 3.10)
project(Error_state_kalman_filter)

set(CMAKE_CXX_STANDARD 17)

option(BUILD_DEMO "Build demo" ON)

set(IMU_FILE_PATH ${CMAKE_SOURCE_DIR}/data/)
configure_file(${CMAKE_SOURCE_DIR}/demo/config.h.in ${CMAKE_SOURCE_DIR}/demo/config.h)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(demo/EmbeddedMath)


if (BUILD_DEMO)
    add_executable(eskf_demo demo/demo.cpp)
endif (BUILD_DEMO)


cmake_minimum_required(VERSION 3.16.3)
project(f9p_relay)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(Threads REQUIRED)

include_directories(include/mavlink)
include_directories(Drivers/src)

add_library(mavlink INTERFACE)
target_include_directories(mavlink INTERFACE include/mavlink)

add_library(gps
        Drivers/src/ubx.cpp
        Drivers/src/gps_helper.cpp
        Drivers/src/rtcm.cpp
)

add_library(serial
        Serial.cpp
)

add_library(mavlink_stream
        MavlinkStream.cpp
)

add_executable(f9p_relay
        F9P.cpp
)

target_link_libraries(f9p_relay
        gps
        mavlink
        serial
        mavlink_stream
        gtest
        gtest_main
        Threads::Threads
)

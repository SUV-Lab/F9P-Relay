cmake_minimum_required(VERSION 3.10)
project(f9p_relay)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(Threads REQUIRED)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/third_party/mavlink)
include_directories(${CMAKE_SOURCE_DIR}/drivers/src)

# MAVLink library (header-only)
add_library(mavlink INTERFACE)
target_include_directories(mavlink INTERFACE ${CMAKE_SOURCE_DIR}/third_party/mavlink)

# GPS driver library
add_library(gps
        drivers/src/ubx.cpp
        drivers/src/gps_helper.cpp
        drivers/src/rtcm.cpp
)
target_include_directories(gps PUBLIC ${CMAKE_SOURCE_DIR}/include)

# Serial interface library
add_library(serial
        src/interface/serial.cpp
)
target_include_directories(serial PUBLIC ${CMAKE_SOURCE_DIR}/include)

# MAVLink stream library
add_library(mavlink_stream
        src/interface/mavlink_stream.cpp
)
target_include_directories(mavlink_stream PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(mavlink_stream serial)

# F9P GPS library
add_library(f9p
        src/gps/f9p.cpp
)
target_include_directories(f9p PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(f9p
        gps
        serial
        mavlink_stream
)

# Main executable
add_executable(f9p_relay
        src/main.cpp
)
target_include_directories(f9p_relay PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(f9p_relay
        f9p
        gps
        mavlink
        serial
        mavlink_stream
        Threads::Threads
)

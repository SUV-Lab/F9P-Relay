
#pragma once

#include <iostream>
#include <cstdint>
#include <thread>
#include <chrono>
#include <sstream> // For std::ostringstream
#include <iomanip> // For std::hex and std::setw


#define GPS_READ_BUFFER_SIZE 1024

#define GPS_INFO(...) do { std::ostringstream oss; oss << __VA_ARGS__; std::cerr << oss.str() << std::endl; } while(0)
#define GPS_WARN(...) do { std::ostringstream oss; oss << __VA_ARGS__; std::cerr << oss.str() << std::endl; } while(0)
#define GPS_ERR(...)  do { std::ostringstream oss; oss << __VA_ARGS__; std::cerr << oss.str() << std::endl; } while(0)


// You may need to include or define vehicle_gps_position and satellite_info
// based on your project structure.
 #include "f9p/types/sensor_gps.h"
 #include "f9p/types/satellite_info.h"
 #include "f9p/types/sensor_gnss_relative.h"

#define M_DEG_TO_RAD (M_PI / 180.0)
#define M_RAD_TO_DEG (180.0 / M_PI)
#define M_DEG_TO_RAD_F 0.01745329251994f
#define M_RAD_TO_DEG_F 57.2957795130823f

class Sleeper {
public:
    static void usleep(unsigned long usecs) {
        std::this_thread::sleep_for(std::chrono::microseconds(usecs));
    }
};

static inline void gps_usleep(unsigned long usecs) {
    Sleeper::usleep(usecs);
}

typedef uint64_t gps_abstime;

static inline gps_abstime gps_absolute_time() {
    auto now = std::chrono::high_resolution_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(now).count();
}

#if defined(_WIN32) && (_MSC_VER < 1900)
struct timespec {
    time_t tv_sec;
    long tv_nsec;
};
#endif

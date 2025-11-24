# F9P-relay

GPS RTCM relay for F9P modules with MAVLink streaming over UDP and LoRa.

## Features

- Real-time RTCM data relay from F9P GPS modules
- Dual output: UDP broadcast and LoRa serial
- MAVLink protocol support
- Test mode for hardware-independent testing

## How to Build

```shell
mkdir build && cd build
cmake ..
make
```

## Usage

### Basic Usage
```shell
./f9p_relay
```

### Command Line Options
```shell
./f9p_relay [-i ip] [-p port] [-l lora_device] [-g f9p_device] [-t]

Options:
  -i ip           UDP broadcast IP (default: 10.42.0.255)
  -p port         UDP port (default: 9750)
  -l lora_device  LoRa device path (default: /dev/ttyUSB0)
  -g f9p_device   F9P GPS device path (default: /dev/ttyACM0)
  -t              Test mode - send dummy RTCM data without GPS hardware
```

### Test Mode

Test mode is useful for testing LoRa and WiFi/UDP transmission without GPS hardware:

```shell
./f9p_relay -t
```

This will:
- Skip GPS device initialization
- Send dummy RTCM data packets (150 bytes) every second
- Allow you to verify that UDP and LoRa transmission paths are working correctly

## Project Structure

```
.
├── src/                    # Source files
│   ├── main.cpp           # Program entry point
│   ├── gps/               # GPS module
│   │   └── f9p.cpp
│   └── interface/         # Communication interfaces
│       ├── serial.cpp
│       └── mavlink_stream.cpp
├── include/f9p/           # Header files
│   ├── gps/
│   ├── interface/
│   └── types/
├── drivers/               # External GPS drivers
└── third_party/           # Third-party libraries (MAVLink)
```

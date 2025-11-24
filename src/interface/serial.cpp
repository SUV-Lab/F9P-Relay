//
// Created by woons on 23. 9. 25.
//

#include "f9p/interface/serial.hpp"

#include <utility>
#include <cstring>
#include <cerrno>

using namespace WSerial;

bool Serial::open() {
    _fd = ::open(_portName.c_str(), O_RDWR | O_NOCTTY /*| O_NDELAY*/  /*O_NONBLOCK*/);
    if (_fd < 0) {
        _lastError = errno;
        cout << "FAIL: Error opening [" << _portName << "] port\n";
        return false;
    }

    tcgetattr(_fd, &_serialConfig);

    _serialConfig.c_cflag |= CLOCAL | CREAD | CS8;
    _serialConfig.c_iflag = IGNPAR;
    _serialConfig.c_oflag = 0;
    _serialConfig.c_lflag = 0;

    if (cfsetispeed(&_serialConfig, _baudRate) < 0 || cfsetospeed(&_serialConfig, _baudRate) < 0) {
        cout << "FAIL: Error setting baudrate / termios config for cfsetispeed\n";
        _lastError = errno;
        return false;
    }

    if (tcsetattr(_fd, TCSANOW, &_serialConfig) < 0) {
        cout << "FAIL: Error setting baudrate / termios config for tcsetattr\n";
        _lastError = errno;
        return false;
    }

    return true;
}

Serial::~Serial() {
    if (_fd >= 0) {
        ::close(_fd);
        _fd = -1;
    }
}

void Serial::setPortName(string portName) {
    _portName = std::move(portName);
}

bool Serial::setBaudRate(const int baudRate) {
    _baudRate = baudRate;
    switch (baudRate) {
        case 9600:
            _baudRate = B9600;
            break;
        case 57600:
            _baudRate = B57600;
            break;
        case 38400:
            _baudRate = B38400;
            break;
        case 115200:
            _baudRate = B115200;
            break;
        case 230400:
            _baudRate = B230400;
            break;
        default:
            cerr << "Unsupported baud rate " << baudRate << "\n";
            return false;
    }
    return true;
}

ssize_t Serial::bytesAvailable() {
    int bytes;
    if (ioctl(_fd, FIONREAD, &bytes) < 0) {
        _lastError = errno;
        cerr << "Error getting available bytes: " << strerror(errno) << "\n";
        return -1;
    }
    return bytes;
}

bool Serial::waitForReadyRead(int timeout) const {
    fd_set set;
    struct timeval timeoutStruct{};

    FD_ZERO(&set);
    FD_SET(_fd, &set);

    timeoutStruct.tv_sec = timeout / 1000;
    timeoutStruct.tv_usec = (timeout % 1000) * 1000;

    return select(_fd + 1, &set, nullptr, nullptr, &timeoutStruct) > 0;
}

ssize_t Serial::read(char *buffer, size_t size) {
    ssize_t bytesRead = ::read(_fd, buffer, size);
    if (bytesRead < 0) {
        _lastError = errno;
        cerr << "Error reading: " << strerror(errno) << "\n";
    }
    return bytesRead;
}

ssize_t Serial::write(const char *buffer, size_t size) {
    ssize_t bytesWritten = ::write(_fd, buffer, size);
    if (bytesWritten < 0) {
        _lastError = errno;
        cerr << "Error writing: " << strerror(errno) << "\n";
    }
    return bytesWritten;
}

bool Serial::waitForBytesWritten(int timeout) {
    auto start = chrono::high_resolution_clock::now();
    while (tcdrain(_fd) != 0) {
        if (chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - start).count() > timeout) {
            _lastError = errno;
            return false;
        }
        this_thread::sleep_for(chrono::milliseconds(1));
    }
    return true;
}

string Serial::errorString() const {
    switch (_lastError) {
        case NoError:
            return "No error occurred.";
        case DeviceNotFoundError:
            return "Failed to open non-existing device.";
        case PermissionError:
            return "Permission denied for device.";
        case OpenError:
            return "Device is already open in this object.";
        case NotOpenError:
            return "Operation can only be performed when the device is open.";
        case WriteError:
            return "An I/O error occurred while writing the data.";
        case ReadError:
            return "An I/O error occurred while reading the data.";
        case ResourceError:
            return "An I/O error occurred when a resource becomes unavailable.";
        case UnsupportedOperationError:
            return "The requested device operation is not supported.";
        case TimeoutError:
            return "A timeout error occurred.";
        case UnknownError:
            return "An unidentified error occurred.";
        default:
            return "Undefined error.";
    }
}

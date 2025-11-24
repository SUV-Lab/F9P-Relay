//
// Created by woons on 23. 9. 25.
//
#pragma once

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>  // for read, write
#include <sys/ioctl.h>  // for ioctl
#include <chrono>  // for std::this_thread::sleep_for
#include <thread>  // for std::this_thread::sleep_for

using namespace std;
namespace WSerial {
    class Serial {
    public:
        enum Error {
            NoError = 0, // No error occurred.
            DeviceNotFoundError, // An error occurred while attempting to open a non-existing device.
            PermissionError, // An error occurred while attempting to open an already opened device by another process or a user not having enough permission and credentials to open.
            OpenError, // An error occurred while attempting to open an already opened device in this object.
            NotOpenError, // This error occurs when an operation is executed that can only be successfully performed if the device is open.
            WriteError, // An I/O error occurred while writing the data.
            ReadError, // An I/O error occurred while reading the data.
            ResourceError, // An I/O error occurred when a resource becomes unavailable, e.g. when the device is unexpectedly removed from the system.
            UnsupportedOperationError, // The requested device operation is not supported or prohibited by the running operating system.
            TimeoutError, // A timeout error occurred.
            UnknownError, // An unidentified error occurred.
        };

        Serial() :
                _fd(-1),
                _baudRate(B115200),
                _lastError(NoError) {}

        ~Serial();

        bool open();

        void setPortName(string portName);

        bool setBaudRate(const int baudRate);

        int error() const { return _lastError; }

        void clearError() { _lastError = NoError; }

        string errorString() const;

        ssize_t bytesAvailable();

        bool waitForReadyRead(int timeout) const;

        ssize_t read(char *buffer, size_t size);

        ssize_t write(const char *buffer, size_t size);

        bool waitForBytesWritten(int timeout);

    private:
        int _fd;
        string _portName;
        int _baudRate;
        struct termios _serialConfig{};
        int _lastError;
    };
}
#include "f9p/gps/f9p.hpp"

void F9P::run() {
    if (_testMode) {
        runTestMode();
        return;
    }

    if (_f9p) delete _f9p;

    // Initial device connection with retry logic
    _f9p = new WSerial::Serial();
    _f9p->setPortName(_device);
    if (!_f9p->open()) {
        int retries = 60;
        // Give the device some time to come up. In some cases the device is not
        // immediately accessible right after startup for some reason. This can take 10-20s.
        cout << "F9P device [" << _device << "] not accessible, retrying...\n";
        while (retries-- > 0 && !_requestStop) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            if (_f9p->open()) {
                cout << "Successfully opened F9P device [" << _device << "]\n";
                _f9p->clearError();
                break;
            }

            // Log specific error types
            if (retries % 10 == 0) {  // Log every 5 seconds
                cout << "Still trying to open device... (" << retries << " retries left) - Error: "
                     << _f9p->errorString() << "\n";
            }
        }
        if (_f9p->error() != WSerial::Serial::NoError) {
            cout << "========= Failed to open F9P Device =========\n";
            cout << "Device: " << _device << "\n";
            cout << "Error: " << _f9p->errorString() << "\n";
            throw std::runtime_error("Failed to initialize F9P");
        }
    }

    cout << "F9P device initialized successfully\n";
    _f9p->setBaudRate(115200);

    unsigned int baudrate;
    GPSBaseStationSupport *gpsDriver = nullptr;

    _sendRTCMThread = std::thread(&F9P::sendRTCM, this);

    while (!_requestStop) {

        if (gpsDriver) {
            delete gpsDriver;
            gpsDriver = nullptr;
        }
        gpsDriver = new GPSDriverUBX(GPSDriverUBX::Interface::UART, &callbackEntry, this, &_reportGpsPos,
                                     _pReportSatInfo);
        baudrate = 0; // auto-configure
        gpsDriver->setSurveyInSpecs(_surveyInAccMeters * 10000.0f, _surveyInDurationSecs);

        if (_useFixedBaseLoction) {
            gpsDriver->setBasePosition(_fixedBaseLatitude, _fixedBaseLongitude, _fixedBaseAltitudeMeters,
                                       _fixedBaseAccuracyMeters * 1000.0f);
        }

        _gpsConfig.output_mode = GPSHelper::OutputMode::RTCM;
        if (gpsDriver->configure(baudrate, _gpsConfig) != 0) {
            // GPS driver configuration failed
            cout << "GPS driver configuration failed, checking serial port status...\n";
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;  // Retry initialization
        }

        /* reset report */
        memset(&_reportGpsPos, 0, sizeof(_reportGpsPos));
        cout << "GPS driver configured successfully, starting data reception...\n";

        // Main data reception loop
        // In rare cases it can happen that we get an error from the driver (eg. checksum failure) due to
        // bus errors or buggy firmware. Track consecutive errors.
        int consecutiveErrors = 0;

        while (!_requestStop) {
            int helperRet = gpsDriver->receive(GPS_RECEIVE_TIMEOUT);

            if (helperRet > 0) {
                consecutiveErrors = 0;  // Reset error counter on success

                if (helperRet & 1) {
                    publishGPSPosition();
                }

                if (_pReportSatInfo && (helperRet & 2)) {
                    publishGPSSatellite();
                }
            } else {
                consecutiveErrors++;
                if (consecutiveErrors >= 3) {
                    cout << "3 consecutive receive errors, checking device status...\n";
                    break;  // Exit inner loop to check for errors
                }
            }
        }

        // If we exited due to _requestStop, break outer loop
        if (_requestStop) {
            break;
        }

        // If we're here, we had consecutive errors. Check for serial port errors
        if (_f9p->error() != WSerial::Serial::NoError && _f9p->error() != WSerial::Serial::TimeoutError) {
            cout << "========= Serial port error detected =========\n";
            cout << "Error: " << _f9p->errorString() << "\n";
            cout << "Attempting to reconnect to device [" << _device << "]...\n";

            // Close and cleanup current connection
            if (_f9p) {
                delete _f9p;
                _f9p = nullptr;
            }

            // Wait before reconnection attempt
            std::this_thread::sleep_for(std::chrono::seconds(2));

            // Attempt to reconnect
            _f9p = new WSerial::Serial();
            _f9p->setPortName(_device);
            _f9p->setBaudRate(115200);

            int reconnectRetries = 10;
            bool reconnected = false;

            while (reconnectRetries-- > 0 && !_requestStop) {
                if (_f9p->open()) {
                    cout << "Successfully reconnected to device [" << _device << "]\n";
                    _f9p->clearError();
                    reconnected = true;
                    break;
                }
                cout << "Reconnect attempt failed, retrying... (" << reconnectRetries << " left)\n";
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }

            if (!reconnected) {
                cout << "========= Failed to reconnect to device =========\n";
                cout << "Will retry GPS driver initialization...\n";
            }
            // Continue to next iteration to reinitialize GPS driver
        } else {
            // No serial error, might be GPS data issue
            cout << "Receive errors without serial port error, reinitializing GPS driver...\n";
        }
    }

    // Cleanup
    if (gpsDriver) {
        delete gpsDriver;
        gpsDriver = nullptr;
    }

    cout << "=================== Exiting F9P thread ===================";
}

F9P::F9P(const string &device,
         double surveyInAccMeters,
         int surveyInDurationSecs,
         const std::atomic_bool &requestStop,
         bool enableSatInfo,
         MavlinkStream *mavlinkStream,
         bool testMode)
        : _mavlinkStream(mavlinkStream), _device(device), _surveyInAccMeters(surveyInAccMeters),
          _surveyInDurationSecs(surveyInDurationSecs), _useFixedBaseLoction(false), _testMode(testMode),
          _requestStop(requestStop), _fixedBaseLatitude(0.0), _fixedBaseLongitude(0.0),
          _fixedBaseAltitudeMeters(0.0), _fixedBaseAccuracyMeters(0.0) {
    if (_testMode) {
        cout << "========= TEST MODE ENABLED =========\n";
        cout << "Running without GPS hardware - sending dummy RTCM data\n";
    } else {
        cout << "Survey in [accuracy:duration]  -> [" << surveyInAccMeters << " : " << surveyInDurationSecs << "]\n";
    }
    if (enableSatInfo) _pReportSatInfo = new satellite_info_s();
}

F9P::~F9P() {
    _cv.notify_all();

    if (_sendRTCMThread.joinable()) {
        _sendRTCMThread.join();
    }

    if (_pReportSatInfo) {
        delete _pReportSatInfo;
        _pReportSatInfo = nullptr;
    }

    if (_f9p) {
        delete _f9p;
        _f9p = nullptr;
    }
}

void F9P::gotRTCMData(uint8_t *data, size_t len) {
    std::vector<uint8_t> message(data, data + len);
    {
        std::lock_guard<std::mutex> lock(_queueMutex);
        _messageQueue.push(message);
    }

    _cv.notify_one();
}

int F9P::callbackEntry(GPSCallbackType type, void *data1, int data2, void *user) {
    F9P *gps = (F9P *) user;
    return gps->callback(type, data1, data2);
}

int F9P::callback(GPSCallbackType type, void *data1, int data2) {
    switch (type) {
        case GPSCallbackType::readDeviceData: {
            if (_f9p->bytesAvailable() == 0) {
                int timeout = *((int *) data1);
                if (!_f9p->waitForReadyRead(timeout))
                    return 0; //timeout
            }
            return (int) _f9p->read((char *) data1, data2);
        }
        case GPSCallbackType::writeDeviceData:
            if (_f9p->write((char *) data1, data2) >= 0) {
                if (_f9p->waitForBytesWritten(-1))
                    return data2;
            }
            return -1;

        case GPSCallbackType::setBaudrate:
            return _f9p->setBaudRate(data2) ? 0 : -1;

        case GPSCallbackType::gotRTCMMessage:
            gotRTCMData((uint8_t *) data1, data2);
            break;

        case GPSCallbackType::surveyInStatus: {
            SurveyInStatus *status = (SurveyInStatus *) data1;
//            cout << "Position: " << status->latitude << status->longitude << status->altitude;
            std::cout << "Survey-in duration: " << status->duration
                      << "s cur accuracy: " << std::fixed << std::setprecision(2)
                      << static_cast<float>(status->mean_accuracy) / 1000.0f
                      << "m valid: " << static_cast<int>(status->flags & 1)
                      << " active: " << static_cast<int>((status->flags >> 1) & 1);
            if (_pReportSatInfo) {
                std::cout << " used satellite: " << static_cast<int>(_pReportSatInfo->count);
            }
            std::cout << "\n";
        }
            break;

        case GPSCallbackType::setClock:
            /* do nothing */
            break;
    }
    return 0;
}

void F9P::publishGPSPosition() {
//    printf("publishGPSPosition\n");
}

void F9P::publishGPSSatellite() {
//    printf("publishGPSSatellite\n");
}

void F9P::SendDummyRTCM() {
    const size_t len = 150;
    uint8_t dummyData[len];

    for(size_t i = 0; i < len; ++i) {
        dummyData[i] = static_cast<uint8_t>(i + 1);
    }

    this->gotRTCMData(dummyData, len);
}

void F9P::runTestMode() {
    cout << "Starting test mode - sending dummy RTCM data every 1 second\n";

    // Start RTCM sending thread
    _sendRTCMThread = std::thread(&F9P::sendRTCM, this);

    int packetCount = 0;
    while (!_requestStop) {
        SendDummyRTCM();
        packetCount++;
        cout << "Test mode: Sent dummy RTCM packet #" << packetCount << " (150 bytes)\n";

        // Wait 1 second before sending next packet
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    cout << "=================== Exiting F9P test mode ===================\n";
}

void F9P::sendRTCM() {
    while (!_requestStop) {
        std::unique_lock<std::mutex> lock(_queueMutex);
        _cv.wait(lock, [this]{ return !_messageQueue.empty() || _requestStop; });
        while (!_messageQueue.empty()) {
            // std::cout << "_messageQueue(" << _messageQueue.size() << ")\n";
            auto message = _messageQueue.front();
            _messageQueue.pop();
            lock.unlock();

            if (_mavlinkStream) {
                _mavlinkStream->RTCMDataUpdate(message);
            }

            lock.lock();
        }
    }
}

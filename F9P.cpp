#include "F9P.hpp"

int main(int argc, char **argv) {
    string udp_ip = "10.42.0.255";
    int udp_port = 9750;
    string lora_device = "/dev/ttyUSB0";
    string f9p_device = "/dev/ttyACM0";

    int option;
    while ((option = getopt(argc, argv, "i:p:l:g:")) != -1) {
        switch (option) {
            case 'i':
                udp_ip = optarg;
                break;
            case 'p':
                udp_port = std::stoi(optarg);
                break;
            case 'l':
                lora_device = optarg;
                break;
            case 'g':
                f9p_device = optarg;
                break;
            case '?': // Unrecognized option
                std::cerr << "Usage: " << argv[0] << " [-i ip] [-p port] [-l lora_device] [-g f9p_device]" << std::endl;
                return 1;
        }
    }

    try {
        MavlinkStream mavlinkStream(udp_ip, udp_port, lora_device);

        F9P f9p(f9p_device, SURVEY_ACC_METER, SURVEY_DURATION_SECS, false, true, &mavlinkStream);
        f9p.run();
    }
    catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1; // Return an error code
    }
    return 0;
}

void F9P::run() {
    if (_f9p) delete _f9p;

    _f9p = new WSerial::Serial();
    _f9p = new WSerial::Serial();
    _f9p->setPortName(_device);
    if (!_f9p->open()) {
        int retries = 60;
        // Give the device some time to come up. In some cases the device is not
        // immediately accessible right after startup for some reason. This can take 10-20s.
        while (retries-- > 0 && _f9p->error() == WSerial::Serial::PermissionError) { // permission error
            cout << "Cannot open device... retrying\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            if (_f9p->open()) {
                _f9p->clearError();
                break;
            }
        }
        if (_f9p->error() != WSerial::Serial::NoError) {
            cout << "========= Failed to open F9P Device =========\n" << _device << _f9p->errorString();
            throw std::runtime_error("Failed to initialize F9P");
        }
    }
    _f9p->setBaudRate(115200);
//    _f9p->setDataBits(QSerialPort::Data8);
//    _f9p->setParity(QSerialPort::NoParity);
//    _f9p->setStopBits(QSerialPort::OneStop);
//    _f9p->setFlowControl(QSerialPort::NoFlowControl);

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
        gpsDriver->setSurveyInSpecs(_surveyInAccMeters * 10000.0f, _surveryInDurationSecs);

        if (_useFixedBaseLoction) {
            gpsDriver->setBasePosition(_fixedBaseLatitude, _fixedBaseLongitude, _fixedBaseAltitudeMeters,
                                       _fixedBaseAccuracyMeters * 1000.0f);
        }

        _gpsConfig.output_mode = GPSHelper::OutputMode::RTCM;
        if (gpsDriver->configure(baudrate, _gpsConfig) == 0) {

            /* reset report */
            memset(&_reportGpsPos, 0, sizeof(_reportGpsPos));

            //In rare cases it can happen that we get an error from the driver (eg. checksum failure) due to
            //bus errors or buggy firmware. In this case we want to try multiple times before giving up.
            int numTries = 0;

            while (!_requestStop && numTries < 3) {
                int helperRet = gpsDriver->receive(GPS_RECEIVE_TIMEOUT);

                if (helperRet > 0) {
                    numTries = 0;

                    if (helperRet & 1) {
                        publishGPSPosition();
                        numTries = 0;
                    }

                    if (_pReportSatInfo && (helperRet & 2)) {
                        publishGPSSatellite();
                        numTries = 0;
                    }
                } else {
                    ++numTries;
                }
            }
            if (_f9p->error() != WSerial::Serial::NoError && _f9p->error() != WSerial::Serial::TimeoutError) {
                break;
            }
        }
    }
    cout << "=================== Exiting F9P thread ===================";
}

F9P::F9P(const string &device,
         double surveyInAccMeters,
         int surveryInDurationSecs,
         const std::atomic_bool &requestStop,
         bool enableSatInfo,
         MavlinkStream *mavlinkStream)
        : _mavlinkStream(mavlinkStream), _device(device), _surveyInAccMeters(surveyInAccMeters),
          _surveryInDurationSecs(surveryInDurationSecs), _useFixedBaseLoction(false), _requestStop(requestStop),
          _fixedBaseLatitude(0.0), _fixedBaseLongitude(0.0), _fixedBaseAltitudeMeters(0.0),
          _fixedBaseAccuracyMeters(0.0) {
    cout << "Survey in [accuracy:duration]  -> [" << surveyInAccMeters << " : " << surveryInDurationSecs << "]\n";
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
                      << " active: " << static_cast<int>((status->flags >> 1) & 1)
                      << " used satellite: " << static_cast<int>(_pReportSatInfo->count) << "\n";
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

void F9P::sendRTCM() {
    while (!_requestStop) {
        std::unique_lock<std::mutex> lock(_queueMutex);
        _cv.wait(lock, [this]{ return !_messageQueue.empty() || _requestStop; });
//        std::cout << "_messageQueue(" << _messageQueue.size() << ")\n";
        while (!_messageQueue.empty()) {
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

#include <iostream>
#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>
#include "ubx.h"
#include "sbf.h"
#include "ashtech.h"
#include "base_station.h"
#include "gps_helper.h"
#include "f9p/types/definitions.h"
#include "f9p/interface/serial.hpp"
#include "f9p/interface/mavlink_stream.hpp"

using namespace std;

#define SURVEY_ACC_METER 10.0
#define SURVEY_DURATION_SECS 180

#define GPS_RECEIVE_TIMEOUT 1200

class F9P {
public:
    F9P(const string& device,
        double     surveyInAccMeters,
        int        surveyInDurationSecs,
        const std::atomic_bool& requestStop,
        bool       enableSatInfo,
        MavlinkStream* mavlinkStream,
        bool       testMode = false);
    ~F9P();

    void run();
    void runTestMode();

private:
    MavlinkStream* _mavlinkStream;

    std::thread _sendRTCMThread;
    std::queue<std::vector<uint8_t>> _messageQueue;
    std::mutex _queueMutex;
    std::condition_variable _cv;

    void sendRTCM();

    /**
     * callback from the driver for the platform specific stuff
     */
    static int callbackEntry(GPSCallbackType type, void *data1, int data2, void *user);

    int callback(GPSCallbackType type, void *data1, int data2);

    double  _surveyInAccMeters;
    int     _surveyInDurationSecs;
    bool    _useFixedBaseLoction;
    bool    _testMode;
    double  _fixedBaseLatitude;
    double  _fixedBaseLongitude;
    float   _fixedBaseAltitudeMeters;
    float   _fixedBaseAccuracyMeters;
    GPSHelper::GPSConfig _gpsConfig{};
    const std::atomic_bool& _requestStop;

    struct sensor_gps_s        _reportGpsPos;
    struct satellite_info_s    *_pReportSatInfo = nullptr;

    string _device;
    WSerial::Serial *_f9p = nullptr;

    void publishGPSPosition();
    void publishGPSSatellite();
    void gotRTCMData(uint8_t *data, size_t len);

    void SendDummyRTCM();
};

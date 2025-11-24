#include <netinet/in.h>
#include <arpa/inet.h>
#include <vector>
#include <cstring>

#include "f9p/interface/serial.hpp"
#include "common/mavlink.h"

#define LORA_PACKET_MAX 240
#define LORA_PAYLOAD 237
#define LORA_CHANNEL 72

using namespace std;
class ElapsedTimer {
public:
    void start() {
        start_point = std::chrono::high_resolution_clock::now();
    }

    // milliseconds로 경과 시간 반환
    auto elapsed() const {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::high_resolution_clock::now() - start_point).count();
    }

    // seconds로 경과 시간 반환
    auto elapsedSeconds() const {
        return std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::high_resolution_clock::now() - start_point).count();
    }

    auto restart() {
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_point).count();
        start_point = now;
        return elapsed_ms;
    }

private:
    std::chrono::high_resolution_clock::time_point start_point;
};

class MavlinkStream {
public:
    MavlinkStream(const string& ip,
                  int port,
                  const string& lora_device);

    ~MavlinkStream();
    bool initBroadcast(const string& ip, int port);
    bool initLora(const string& device);
    void RTCMDataUpdate(const vector<uint8_t>& message);
    int sendToLora(const mavlink_message_t& msg);
    int sendToUDP(const mavlink_message_t& msg);

private:
    WSerial::Serial *_lora = nullptr;
    int _udpSock;
    struct sockaddr_in _sock_in{};

    ElapsedTimer _bandwidthTimer;
    int _bandwidthByteCounter;
    uint8_t _sequenceId;

    static constexpr uint8_t LORA_HEADER[3] = {0xFF, 0xFF, static_cast<uint8_t>(LORA_CHANNEL)};


    void sendMessageToVehicle(const mavlink_gps_rtcm_data_t &msg);
};

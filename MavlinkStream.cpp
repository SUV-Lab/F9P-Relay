#include "MavlinkStream.hpp"

MavlinkStream::MavlinkStream(const string &udp_ip,
                             int udp_port,
                             const string &lora_device)
        : _bandwidthByteCounter(0), _sequenceId(0), _udpSock(-1) {
    _bandwidthTimer.start();
    if (!initBroadcast(udp_ip, udp_port)) {
        throw std::runtime_error("Failed to initialize UDP Broadcast");
    }

    if (!initLora(lora_device)) {
        cout << "Failed to initialize Lora\n";
        throw std::runtime_error("Failed to initialize Lora");
    }
}

MavlinkStream::~MavlinkStream() {
    if (_lora) { delete _lora; }
    if (_udpSock >= 0) { close(_udpSock); }
}

bool MavlinkStream::initBroadcast(const string &ip, int port) {
    _udpSock = socket(AF_INET, SOCK_DGRAM, 0);
    if (_udpSock < 0) {
        std::cerr << "Error opening socket: " << std::strerror(errno) << std::endl;
        return false;
    }

    int broadcastEnable = 1;
    int ret = setsockopt(_udpSock, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));
    if (ret) {
        std::cerr << "Error setting broadcast option: " << std::strerror(errno) << std::endl;
        close(_udpSock);
        _udpSock = -1;
        return false;
    }

    memset(&_sock_in, 0, sizeof(_sock_in));
    _sock_in.sin_family = AF_INET;
    _sock_in.sin_port = htons(port);
    _sock_in.sin_addr.s_addr = inet_addr(ip.c_str());

    return true;
}

bool MavlinkStream::initLora(const string &device) {
    if (_lora) { delete _lora; }
    _lora = new WSerial::Serial();
    _lora->setPortName(device);
    _lora->setBaudRate(115200);
    if (!_lora->open()) {
        int retries = 60;
        // Give the device some time to come up. In some cases the device is not
        // immediately accessible right after startup for some reason. This can take 10-20s.
        while (retries-- > 0 && _lora->error() == WSerial::Serial::PermissionError) { // permission error
            cout << "Cannot open device... retrying\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            if (_lora->open()) {
                _lora->clearError();
                break;
            }
        }
        if (_lora->error() != WSerial::Serial::NoError) {
            cout << "========= Failed to open LORA Device :  [" << device << _lora->errorString() << "]=========\n";
            return false;
        }
    }
    return true;
}

void MavlinkStream::RTCMDataUpdate(std::vector<uint8_t> message) {
    int message_size = static_cast<int>(message.size());
    /* statistics */
    _bandwidthByteCounter += message_size;
    int64_t elapsed = _bandwidthTimer.elapsed();
    if (elapsed > 1000) {
        printf("RTCM bandwidth: %.2f kB/s\n", (float) _bandwidthByteCounter / elapsed * 1000.f / 1024.f);
        _bandwidthTimer.restart();
        _bandwidthByteCounter = 0;
    }

    const int maxMessageLength = MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN;
    mavlink_gps_rtcm_data_t mavlinkRtcmData;
    memset(&mavlinkRtcmData, 0, sizeof(mavlink_gps_rtcm_data_t));

    if (message_size < maxMessageLength) {
        mavlinkRtcmData.len = message_size;
        mavlinkRtcmData.flags = (_sequenceId & 0x1F) << 3;
        memcpy(&mavlinkRtcmData.data, message.data(), message_size);
        sendMessageToVehicle(mavlinkRtcmData);
    } else {
        // We need to fragment

        uint8_t fragmentId = 0;         // Fragment id indicates the fragment within a set
        int start = 0;
        while (start < message_size) {
            int length = std::min(message_size - start, maxMessageLength);
            mavlinkRtcmData.flags = 1;                      // LSB set indicates message is fragmented
            mavlinkRtcmData.flags |= fragmentId++ << 1;     // Next 2 bits are fragment id
            mavlinkRtcmData.flags |= (_sequenceId & 0x1F) << 3;     // Next 5 bits are sequence id
            mavlinkRtcmData.len = length;
            memcpy(&mavlinkRtcmData.data, message.data() + start, length);
            sendMessageToVehicle(mavlinkRtcmData);
            start += length;
        }
    }
    ++_sequenceId;
}

void MavlinkStream::sendMessageToVehicle(const mavlink_gps_rtcm_data_t &msg) {
    mavlink_message_t wifi_message;
    mavlink_message_t lora_message;
    mavlink_msg_gps_rtcm_data_encode_chan(0, 15, 0, &wifi_message, &msg);
    mavlink_msg_gps_rtcm_data_encode_chan(0, 15, 1, &lora_message, &msg);
//    cout << "Send rtcm\n";
    if (_udpSock) {
        sendToUDP(wifi_message);
//        printf("send to UDP(%d)\n", wifi_message.seq);
    } else {
        cout << "=============== Failed send to UDP ===============\n";
    }
    if (_lora) {
        sendToLora(lora_message);
//        printf("send to LoRa(%d)\n", lora_message.seq);
    } else {
        cout << "=============== Failed send to LORA ===============\n";
    }

}

int MavlinkStream::sendToLora(const mavlink_message_t &msg) {
    uint8_t lora_buffer[LORA_PAYLOAD];
    uint8_t packet[LORA_PACKET_MAX];
//    cout << "mavlink size : " << sizeof(msg) << "\n";
    int len = mavlink_msg_to_send_buffer(lora_buffer, &msg);
    memcpy(packet, LORA_HEADER, sizeof(LORA_HEADER));
    memcpy(packet + sizeof(LORA_HEADER), lora_buffer, len);
    int ret = _lora->write(reinterpret_cast<const char *>(packet), len + sizeof(LORA_HEADER));
    auto now = std::chrono::system_clock::now();

//    printf("fdfasf : %d\n", msg.len);
    // 시간을 시간대(time_t)로 변환
    auto usec = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();

    // 시간을 문자열로 변환하여 출력
//    std::cout << "ret : " << ret << "len : "<< len <<" Current time: " << usec << "\n";
    if (ret < 0) {
        perror("Failed to write to serial port\n");
    }
    return 0;
}

int MavlinkStream::sendToUDP(const mavlink_message_t &msg) {
    // Write message into buffer, prepending start sign
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int status, len;
    len = mavlink_msg_to_send_buffer(buffer, &msg);

    status = sendto(_udpSock, buffer, len, 0, (struct sockaddr *) &_sock_in, sizeof(struct sockaddr_in));
    if (status == -1) {
        perror("Failed to send to UDP socket\n");
        return -1;
    }

    return 0;
}

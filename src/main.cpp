#include "f9p/gps/f9p.hpp"
#include <unistd.h>

int main(int argc, char **argv) {
    string udp_ip = "10.42.0.255";
    int udp_port = 9750;
    string lora_device = "/dev/ttyUSB0";
    string f9p_device = "/dev/ttyACM0";
    bool test_mode = false;

    int option;
    while ((option = getopt(argc, argv, "i:p:l:g:t")) != -1) {
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
            case 't':
                test_mode = true;
                break;
            case '?': // Unrecognized option
                std::cerr << "Usage: " << argv[0] << " [-i ip] [-p port] [-l lora_device] [-g f9p_device] [-t]" << std::endl;
                std::cerr << "  -i ip           : UDP broadcast IP (default: 10.42.0.255)" << std::endl;
                std::cerr << "  -p port         : UDP port (default: 9750)" << std::endl;
                std::cerr << "  -l lora_device  : LoRa device path (default: /dev/ttyUSB0)" << std::endl;
                std::cerr << "  -g f9p_device   : F9P GPS device path (default: /dev/ttyACM0)" << std::endl;
                std::cerr << "  -t              : Test mode - send dummy RTCM data without GPS hardware" << std::endl;
                return 1;
        }
    }

    try {
        MavlinkStream mavlinkStream(udp_ip, udp_port, lora_device);

        std::atomic_bool requestStop(false);
        F9P f9p(f9p_device, SURVEY_ACC_METER, SURVEY_DURATION_SECS, requestStop, true, &mavlinkStream, test_mode);
        f9p.run();
    }
    catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1; // Return an error code
    }
    return 0;
}

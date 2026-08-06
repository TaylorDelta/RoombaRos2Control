#include <boost/asio.hpp>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <cstdint>

using boost::asio::serial_port;
using boost::asio::io_service;

static constexpr const char* SERIAL_PORT = "/dev/ttyUSB0"; // "/dev/ttyUSB0" on Linux
static constexpr int BAUD_RATE = 115200;

// Big-endian helpers
int16_t be16(const uint8_t* p) {
    return (int16_t)((p[0] << 8) | p[1]);
}
uint16_t ube16(const uint8_t* p) {
    return (uint16_t)((p[0] << 8) | p[1]);
}

int main() {
    io_service io;
    serial_port ser(io);

    ser.open(SERIAL_PORT);
    ser.set_option(serial_port::baud_rate(BAUD_RATE));
    ser.set_option(serial_port::character_size(8));
    ser.set_option(serial_port::parity(serial_port::parity::none));
    ser.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    ser.set_option(serial_port::flow_control(serial_port::flow_control::none));

    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Start
    boost::asio::write(ser, boost::asio::buffer("\x80", 1));
    std::cout << "Start Roomba\n";

    // Full mode
    boost::asio::write(ser, boost::asio::buffer("\x84", 1));
    std::cout << "Set to Full Mode\n";

    // Start stream: [148, 1, 100]
    uint8_t stream_cmd[] = {148, 1, 100};
    boost::asio::write(ser, boost::asio::buffer(stream_cmd, 3));

    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::vector<uint8_t> buffer(2048);
    int t = 1;

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));

        boost::system::error_code ec;
        size_t n = ser.read_some(
            boost::asio::buffer(buffer.data(), buffer.size()),
            ec
        );

        if (ec || n == 0)
            continue;

        // Scan backwards for packet start (ID 19)
        for (int i = (int)n - 1; i >= 0; --i) {
            if (buffer[i] == 19 && i + 84 <= (int)n) {
                uint8_t* pkt = buffer.data() + i;
                uint8_t checksum = 0;
                for (int k = 0; k < 84; k++) checksum += pkt[k];
                if ((checksum & 0xFF) != 0) continue;

                uint8_t* p = pkt + 3;

                uint8_t bumps = p[0];
                int bump_right = bumps & 1;
                int bump_left = (bumps >> 1) & 1;

                int distance = -be16(&p[12]) * 10;
                double angle = (2.0 * be16(&p[14])) / 235.0;

                uint16_t voltage = ube16(&p[17]);
                int16_t current = be16(&p[19]);
                uint8_t temperature = p[21];
                uint16_t charge = ube16(&p[22]);
                uint16_t capacity = ube16(&p[24]);

                int16_t velocity = be16(&p[44]);
                int16_t vel_r = be16(&p[48]);
                int16_t vel_l = be16(&p[50]);

                uint16_t enc_l = ube16(&p[52]);
                uint16_t enc_r = ube16(&p[54]);

                std::cout
                    << "\rT:" << t
                    << " Bump L:" << bump_left
                    << " R:" << bump_right
                    << " V:" << voltage << "mV"
                    << " Ch:" << charge << "/" << capacity
                    << " Enc L:" << enc_l << " R:" << enc_r
                    << std::flush;
                break;
            }
        }

        // Drive commands
        if (t == 100) {
            uint8_t cmd[] = {0x87};
            boost::asio::write(ser, boost::asio::buffer(cmd, sizeof(cmd)));
            std::cout << "\nStart cleaning\n";



            // uint8_t cmd[] = {0x05, 0x91, 0x00, 0x64, 0x00, 0xC8};
            // boost::asio::write(ser, boost::asio::buffer(cmd, sizeof(cmd)));
            // std::cout << "\nDrive: R=100 L=200\n";
        }

        if (t == 1000) {
            uint8_t cmd[] = {0x80};
            boost::asio::write(ser, boost::asio::buffer(cmd, sizeof(cmd)));
            std::cout << "\nStop cleaning\n";

            // uint8_t cmd[] = {0x05, 0x91, 0x00, 0x00, 0x00, 0x00};
            // boost::asio::write(ser, boost::asio::buffer(cmd, sizeof(cmd)));
            // std::cout << "\nStop drive\n";
        }

        if (t++ > 1100) break;
    }

    // Stop stream
    uint8_t stop_stream[] = {150, 0};
    boost::asio::write(ser, boost::asio::buffer(stop_stream, 2));

    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Stop Roomba
    boost::asio::write(ser, boost::asio::buffer("\xAD", 1));
    std::cout << "\nStopping Roomba...\n";

    ser.close();
    return 0;
}

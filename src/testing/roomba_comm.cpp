#include <iostream>
#include <fcntl.h>      // O_RDWR, O_NOCTTY, O_NONBLOCK
#include <termios.h>    // struct termios, tcgetattr, tcsetattr, baud rates
#include <unistd.h>     // write(), close(), usleep()
#include <cstring>      // memset
#include <cstdint>      // uint8_t
#include <chrono>       // std::chrono
#include <thread>       // std::this_thread::sleep_for

using namespace std;

// Setup the serial port (adjust the port name accordingly)
const string SERIAL_PORT = "/dev/ttyUSB0";  // Change to your serial port (e.g., COMx on Windows)
const uint32_t BAUD_RATE = B115200;

// Function to configure the serial port
int setup_serial(const string& port) {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);  // Open serial port
    if (fd == -1) {
        cerr << "Error opening serial port!" << endl;
        return -1;
    }

    // Get current terminal attributes
    struct termios options;
    if (tcgetattr(fd, &options) < 0) {
        cerr << "Error getting terminal attributes!" << endl;
        close(fd);
        return -1;
    }

    // Set baud rate
    cfsetispeed(&options, BAUD_RATE);
    cfsetospeed(&options, BAUD_RATE);

    // Set the serial port to raw mode
    options.c_cflag &= ~PARENB;    // No parity
    options.c_cflag &= ~CSTOPB;    // One stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;        // 8 data bits
    options.c_cflag |= CREAD | CLOCAL; // Enable receiver, disable modem control lines

    // Apply the new settings
    if (tcsetattr(fd, TCSANOW, &options) < 0) {
        cerr << "Error setting terminal attributes!" << endl;
        close(fd);
        return -1;
    }

    // Flush any remaining data in the buffers
    tcflush(fd, TCIOFLUSH);

    return fd;
}

// Function to send a command to Roomba
void send_command(int fd, uint8_t command) {
    write(fd, &command, 1);
    usleep(10000);  // Give Roomba time to process the command
}

// Function to send the Start command (opcode 128)
void start_roomba(int fd) {
    cout << "Sending Start command..." << endl;
    send_command(fd, 0x80);  // Opcode 128 (Start SCI)
    this_thread::sleep_for(chrono::seconds(1));  // Wait for Roomba to initialize
}

// Function to request sensor data from Roomba
void request_sensor_data(int fd) {
    cout << "Requesting sensor data..." << endl;

    send_command(fd, 0x8E);  // Opcode 142 (Sensors)
    send_command(fd, 0x00);  // Packet Code 0 (All sensor data)

    // Read the response (maximum 26 bytes for packet code 0)
    uint8_t response[26];
    int bytesRead = 0;

    // Set up a timeout loop to read data from the serial port
    auto start_time = chrono::steady_clock::now();
    while (bytesRead < 26) {
        bytesRead = read(fd, response + bytesRead, sizeof(response) - bytesRead);

        // Check if we've waited too long for the response
        auto current_time = chrono::steady_clock::now();
        if (chrono::duration_cast<chrono::seconds>(current_time - start_time).count() > 2) {
            cerr << "Error: Timeout while reading sensor data." << endl;
            return;
        }
    }

    // Print raw response for debugging
    cout << "Raw sensor data: ";
    for (int i = 0; i < 26; ++i) {
        cout << "0x" << hex << static_cast<int>(response[i]) << " ";
    }
    cout << endl;

    // Parse the sensor data (convert to double where needed)
    uint8_t bumps_and_wheeldrops = response[0];
    double bump_right = static_cast<double>(bumps_and_wheeldrops & 1);  // Bit 0: Bump Right
    double bump_left = static_cast<double>((bumps_and_wheeldrops >> 1) & 1);  // Bit 1: Bump Left
    double wheel_drop_right = static_cast<double>((bumps_and_wheeldrops >> 2) & 1);  // Bit 2: Wheel Drop Right
    double wheel_drop_left = static_cast<double>((bumps_and_wheeldrops >> 3) & 1);  // Bit 3: Wheel Drop Left
    double wheel_drop_caster = static_cast<double>((bumps_and_wheeldrops >> 4) & 1);  // Bit 4: Wheel Drop Caster

    double wall = static_cast<double>(response[1]);  // 1 byte, wall sensor
    double cliff_left = static_cast<double>(response[2]);  // 1 byte, left cliff sensor
    double cliff_front_left = static_cast<double>(response[3]);  // 1 byte, front left cliff sensor
    double cliff_front_right = static_cast<double>(response[4]);  // 1 byte, front right cliff sensor
    double cliff_right = static_cast<double>(response[5]);  // 1 byte, right cliff sensor
    double virtual_wall = static_cast<double>(response[6]);  // 1 byte, virtual wall sensor
    uint8_t motor_overcurrents = response[7];
    double side_brush = static_cast<double>(motor_overcurrents & 1);  // Bit 0: Side Brush
    double vacuum = static_cast<double>((motor_overcurrents >> 1) & 1);  // Bit 1: Vacuum
    double main_brush = static_cast<double>((motor_overcurrents >> 2) & 1);  // Bit 2: Main Brush
    double drive_right = static_cast<double>((motor_overcurrents >> 3) & 1);  // Bit 3: Drive Right
    double drive_left = static_cast<double>((motor_overcurrents >> 4) & 1);  // Bit 4: Drive Left

    double dirt_left = static_cast<double>(response[8]);  // 1 byte, dirt detector left
    double dirt_right = static_cast<double>(response[9]);  // 1 byte, dirt detector right
    double remote_control = static_cast<double>(response[10]);  // 1 byte, remote control command
    double buttons = static_cast<double>(response[11]);  // 1 byte, button states

    // Unpack 2-byte signed distance (mm) and angle (mm)
    double distance = static_cast<double>((static_cast<int16_t>(response[12]) << 8) | response[13]);  // 2 bytes, signed distance (mm)
    double angle = static_cast<double>((static_cast<int16_t>(response[14]) << 8) | response[15]);  // 2 bytes, signed angle (mm)

    double charging_state = static_cast<double>(response[16]);  // 1 byte, charging state

    // Map charging state to a description (handle charging_state as integer for the switch)
    const char* charging_state_description = "Unknown";
    switch (static_cast<int>(charging_state)) {
        case 0: charging_state_description = "Not Charging"; break;
        case 1: charging_state_description = "Charging Recovery"; break;
        case 2: charging_state_description = "Charging"; break;
        case 3: charging_state_description = "Trickle Charging"; break;
        case 4: charging_state_description = "Waiting"; break;
        case 5: charging_state_description = "Charging Error"; break;
    }

    // Unpack 2-byte unsigned values (voltage, current, charge, capacity)
    double voltage = static_cast<double>((static_cast<uint16_t>(response[17]) << 8) | response[18]);  // 2 bytes, unsigned voltage (mV)
    double current = static_cast<double>((static_cast<int16_t>(response[19] << 8) | response[20]));  // 2 bytes, signed current (mA)
    double temperature = static_cast<double>(response[21]);  // 1 byte, temperature (°C)
    double charge = static_cast<double>((static_cast<uint16_t>(response[22]) << 8) | response[23]);  // 2 bytes, unsigned charge (mAh)
    double capacity = static_cast<double>((static_cast<uint16_t>(response[24]) << 8) | response[25]);  // 2 bytes, unsigned capacity (mAh)

    // Print parsed data (now all values are double)
    cout << "Bumps and Wheeldrops:" << endl;
    cout << "  Bump Left: " << bump_left << endl;
    cout << "  Bump Right: " << bump_right << endl;
    cout << "  Wheel Drop Left: " << wheel_drop_left << endl;
    cout << "  Wheel Drop Right: " << wheel_drop_right << endl;
    cout << "  Wheel Drop Caster: " << wheel_drop_caster << endl;

    cout << "Wall Sensor: " << wall << endl;
    cout << "Cliff Sensors:" << endl;
    cout << "  Cliff Left: " << cliff_left << endl;
    cout << "  Cliff Front Left: " << cliff_front_left << endl;
    cout << "  Cliff Front Right: " << cliff_front_right << endl;
    cout << "  Cliff Right: " << cliff_right << endl;

    cout << "Virtual Wall: " << virtual_wall << endl;

    cout << "Motor Overcurrents:" << endl;
    cout << "  Side Brush: " << side_brush << endl;
    cout << "  Vacuum: " << vacuum << endl;
    cout << "  Main Brush: " << main_brush << endl;
    cout << "  Drive Right: " << drive_right << endl;
    cout << "  Drive Left: " << drive_left << endl;

    cout << "Dirt Detectors:" << endl;
    cout << "  Dirt Left: " << dirt_left << endl;
    cout << "  Dirt Right: " << dirt_right << endl;

    cout << "Remote Control Command: " << hex << uppercase << static_cast<int>(remote_control) << endl;  // Format as uppercase hex
    cout << "Buttons Pressed: " << buttons << endl;

    cout << "Distance Traveled: " << distance << " mm" << endl;
    cout << "Angle Turned: " << angle << " mm" << endl;

    cout << "Charging State: " << charging_state_description << endl;

    cout << "Battery Information:" << endl;
    cout << "  Voltage: " << voltage << " mV" << endl;
    cout << "  Current: " << current << " mA" << endl;  // Correctly display signed current
    cout << "  Temperature: " << temperature << " °C" << endl;
    cout << "  Charge: " << charge << " mAh" << endl;  // Display charge in decimal
    cout << "  Capacity: " << capacity << " mAh" << endl;  // Display capacity in decimal
}


// Main function
int main() {
    cout << "Connecting to Roomba..." << endl;

    // Setup serial connection
    int fd = setup_serial(SERIAL_PORT);
    if (fd == -1) {
        return -1;
    }

    // Send Start command to initialize the SCI
    start_roomba(fd);

    // Request sensor data after SCI is started
    request_sensor_data(fd);

    // Close the serial port
    close(fd);
    return 0;
}
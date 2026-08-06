from numpy import pi
import serial
import time
import struct

# Setup the serial port (adjust the port name accordingly)
SERIAL_PORT = '/dev/ttyUSB0'  # Change to your serial port (e.g., COMx on Windows)
BAUD_RATE = 115200

# Create a serial connection to Roomba
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# Function to send a command to Roomba
def send_command(command):
    """Send a single byte command to Roomba."""
    ser.write(command)
    time.sleep(0.01)  # Give Roomba time to process the command

# Function to send the Start command (opcode 128)
def start_roomba():
    """Send the Start SCI command to initialize the SCI interface."""
    print("Sending Start command...")
    send_command(b'\x80')  # Opcode 128 (Start SCI)
    time.sleep(1)  # Wait for Roomba to initialize



def request_sensor_data():
    """Send the Sensor command to Roomba and read the data."""
    print("Requesting sensor data...")
    send_command(b'\x8E')  # Opcode 142 (Sensors)
    send_command(b'\x00')  # Packet Code 0 (All sensor data)

    # Read the response (maximum 26 bytes for packet code 0)
    response = ser.read(26)
    if len(response) != 26:
        print("Error: Received incomplete data.")
        return None
    
    # Print raw response for debugging
    print("Raw sensor data:", list(response))
    
    # Parse the sensor data
    bumps_and_wheeldrops = response[0]
    bump_right = bumps_and_wheeldrops & 1  # Bit 0: Bump Right
    bump_left = (bumps_and_wheeldrops >> 1) & 1  # Bit 1: Bump Left
    wheel_drop_right = (bumps_and_wheeldrops >> 2) & 1  # Bit 2: Wheel Drop Right
    wheel_drop_left = (bumps_and_wheeldrops >> 3) & 1  # Bit 3: Wheel Drop Left
    wheel_drop_caster = (bumps_and_wheeldrops >> 4) & 1  # Bit 4: Wheel Drop Caster

    wall = response[1]  # 1 byte, wall sensor
    cliff_left = response[2]  # 1 byte, left cliff sensor
    cliff_front_left = response[3]  # 1 byte, front left cliff sensor
    cliff_front_right = response[4]  # 1 byte, front right cliff sensor
    cliff_right = response[5]  # 1 byte, right cliff sensor
    virtual_wall = response[6]  # 1 byte, virtual wall sensor
    motor_overcurrents = response[7]
    side_brush = motor_overcurrents & 1  # Bit 0: Side Brush
    vaccum = (motor_overcurrents >> 1) & 1  # Bit 1: Vacuum
    main_brush = (motor_overcurrents >> 2) & 1  # Bit 2: Main Brush
    drive_right = (motor_overcurrents >> 3) & 1  # Bit 3: Drive Right
    drive_left = (motor_overcurrents >> 4) & 1  # Bit 4: Drive Left

    dirt_left = response[8]  # 1 byte, dirt detector left
    dirt_right = response[9]  # 1 byte, dirt detector right
    remote_control = response[10]  # 1 byte, remote control command
    buttons = response[11]  # 1 byte, button states

    # Unpack 2-byte signed distance (mm) and angle (mm)
    distance = -struct.unpack('>h', response[12:14])[0]*10  # 2 bytes, signed distance (mm)
    angle = (2*struct.unpack('>h', response[14:16])[0])/235  # 2 bytes, signed angle (mm)

    charging_state = response[16]  # 1 byte, charging state

    # Map charging state to a description
    charging_state_mapping = {
        0: "Not Charging",
        1: "Charging Recovery",
        2: "Charging",
        3: "Trickle Charging",
        4: "Waiting",
        5: "Charging Error"
    }
    charging_state_description = charging_state_mapping.get(charging_state, "Unknown")

    # Unpack 2-byte unsigned values (voltage, current, charge, capacity)
    voltage = struct.unpack('>H', response[17:19])[0]  # 2 bytes, unsigned voltage (mV)
    current = struct.unpack('>h', response[19:21])[0]  # 2 bytes, signed current (mA)
    temperature = response[21]  # 1 byte, temperature (°C)
    charge = struct.unpack('>H', response[22:24])[0]  # 2 bytes, unsigned charge (mAh)
    capacity = struct.unpack('>H', response[24:26])[0]  # 2 bytes, unsigned capacity (mAh)

    # Print parsed data
    print(f"Bumps and Wheeldrops - Bump Left:", bump_left, "Bump Right:", bump_right, "Wheel Drop Left:", wheel_drop_left, "Wheel Drop Right:", wheel_drop_right, "Wheel Drop Caster:", wheel_drop_caster)   
    print(f"Wall Sensor: {wall}")
    print(f"Cliff Left: {cliff_left}")
    print(f"Cliff Front Left: {cliff_front_left}")
    print(f"Cliff Front Right: {cliff_front_right}")
    print(f"Cliff Right: {cliff_right}")
    print(f"Virtual Wall: {virtual_wall}")
    print(f"Motor Overcurrents - Side Brush:", side_brush, "Vacuum:", vaccum, "Main Brush:", main_brush, "Drive Right:", drive_right, "Drive Left:", drive_left)
    print(f"Dirt Left: {dirt_left}")
    print(f"Dirt Right: {dirt_right}")
    print(f"Remote Control Command: {remote_control}")
    print(f"Buttons Pressed: {buttons}")
    print(f"Distance Traveled: {distance} mm")
    print(f"Angle Turned: {angle} mm")
    print(f"Charging State: {charging_state_description}")
    print(f"Battery Voltage: {voltage} mV")
    print(f"Battery Current: {current} mA")
    print(f"Battery Temperature: {temperature} °C")
    print(f"Battery Charge: {charge} mAh")
    print(f"Battery Capacity: {capacity} mAh")

"""Request sensor data packet 101 (28 bytes)."""
def request_sensor_data_packet_101(): 
    print("Requesting sensor data packet 101...")
    send_command(b'\x8E')  # Opcode 142 (Sensors)
    send_command(b'\x65')  # Packet Code 101 (packet 101)

    # Read the response (maximum 28 bytes for packet code 101)
    response = ser.read(28)
    if len(response) != 28:
        print("Error: Received incomplete data.")
        return None
    
    # Print raw response for debugging
    print("Raw sensor data packet 101:", list(response))

    '''
    100 101 43 Encoder Counts Left 2 0 - 65535
100 101 44 Encoder Counts Right 2 0 - 65535
100 101 45 Light Bumper 1 0 - 127
100 101 106 46 Light Bump Left 2 0 - 4095
100 101 106 47 Light Bump Front Left 2 0 - 4095
100 101 106 48 Light Bump Center Left 2 0 - 4095
100 101 106 49 Light Bump Center Right 2 0 - 4095
100 101 106 50 Light Bump Front Right 2 0 - 4095
100 101 106 51 Light Bump Right 2 0 - 4095
100 101 52 Ir Opcode Left 1 0 - 255
100 101 53 Ir Opcode Right 1 0 - 255
100 101 107 54 Left Motor Current 2 -32768 - 32767 mA
100 101 107 55 Right Motor Current 2 -32768 - 32767 mA
100 101 107 56 Main Brush Current 2 -32768 - 32767 mA
100 101 107 57 Side Brush Current 2 -32768 - 32767 mA
100 101 107 58 Stasis 1 0 - 3
    '''
    encoder_counts_left = struct.unpack('>H', response[0:2])[0]
    encoder_counts_right = struct.unpack('>H', response[2:4])[0]
    # calculate distance from encoder counts
    # Constant(TICK_PER_REV=508.8, WHEEL_DIAMETER=72, WHEEL_BASE=235,TICK_TO_DISTANCE=0.44456499814949904317867595046408)
    tick_to_distance = (72*pi)/508.8  # in mm
    distance_left = encoder_counts_left * tick_to_distance
    distance_right = encoder_counts_right * tick_to_distance  # in mm
    light_bumper = response[4]
    light_bump_left = struct.unpack('>H', response[5:7])[0]
    light_bump_front_left = struct.unpack('>H', response[7:9])[0]
    light_bump_center_left = struct.unpack('>H', response[9:11])[0]
    light_bump_center_right = struct.unpack('>H', response[11:13])[0]
    light_bump_front_right = struct.unpack('>H', response[13:15])[0]
    light_bump_right = struct.unpack('>H', response[15:17])[0]
    ir_opcode_left = response[17]       
    ir_opcode_right = response[18]
    left_motor_current = struct.unpack('>h', response[19:21])[0]    
    right_motor_current = struct.unpack('>h', response[21:23])[0]
    main_brush_current = struct.unpack('>h', response[23:25])[0]
    side_brush_current = struct.unpack('>h', response[25:27])[0]
    stasis = response[27]

    # Print parsed data
    print(f"Encoder Counts Left: {encoder_counts_left}")
    print(f"Encoder Counts Right: {encoder_counts_right}")
    print(f"Distance Left: {distance_left} mm")
    print(f"Distance Right: {distance_right} mm")
    print(f"Light Bumper: {light_bumper}")
    print(f"Light Bump Left: {light_bump_left}")        
    print(f"Light Bump Front Left: {light_bump_front_left}")
    print(f"Light Bump Center Left: {light_bump_center_left}")  
    print(f"Light Bump Center Right: {light_bump_center_right}")
    print(f"Light Bump Front Right: {light_bump_front_right}")
    print(f"Light Bump Right: {light_bump_right}")
    print(f"IR Opcode Left: {ir_opcode_left}")
    print(f"IR Opcode Right: {ir_opcode_right}")
    print(f"Left Motor Current: {left_motor_current} mA")
    print(f"Right Motor Current: {right_motor_current} mA")
    print(f"Main Brush Current: {main_brush_current} mA")
    print(f"Side Brush Current: {side_brush_current} mA")
    print(f"Stasis: {stasis}")

def get_distance():
    # Serial sequence: [149][Number of Packets][Packet ID 1][Packet ID 2]...[Packet ID N]
    # send 149 2 43 44
    send_command(b'\x05')  # Length of command sequence
    send_command(b'\x95')  # Opcode 149 (Query List)
    send_command(b'\x02')  # Number of packets
    send_command(b'\x2B')  # Packet ID 43 (Encoder Counts Left)
    send_command(b'\x2C')  # Packet ID 44 (Encoder Counts Right)
    # Read the response (4 bytes: 2 bytes for each encoder count)
    response = ser.read(4)
    if len(response) != 4:
        print("Error: Received incomplete data for distance.")
        return None
    encoder_counts_left = struct.unpack('>H', response[0:2])[0]
    encoder_counts_right = struct.unpack('>H', response[2:4])[0]
    # calculate distance from encoder counts
    tick_to_distance = (72*pi)/508.8  # in mm
    distance_left = encoder_counts_left * tick_to_distance
    distance_right = encoder_counts_right * tick_to_distance  # in mm
    
    overall_distance = (distance_left + distance_right) / 2

    angle = (distance_right - distance_left) / 235 * (180/pi)  # in degrees

    print(f"Distance Left: {distance_left} mm")
    print(f"Distance Right: {distance_right} mm")
    print(f"Overall Distance: {overall_distance} mm")
    print(f"Angle Turned: {angle} degrees")
    
# Main function
def main():
    print("Connecting to Roomba...")
    time.sleep(2)  # Wait for connection to stabilize

    # Send Start command to initialize the SCI
    start_roomba()

    # Request sensor data after SCI is started
    request_sensor_data()

    send_command(b'\x84')  # Opcode 132 Full Mode
    time.sleep(1)


    get_distance()

    #Serial sequence: [145] [Right velocity high byte] [Right velocity low byte] [Left velocity high byte][Left velocity low byte]
    # send left velocity = 200 and right velocity = 100
    # send command length 5, serial.write in one go
    command = bytearray()
    command.append(5)  # Length of command sequence
    command.append(145)  # Opcode 145 (Drive)
    command.append(0)  # Right velocity high byte
    command.append(100)  # Right velocity low byte (100 in decimal)
    command.append(0)  # Left velocity high byte
    ser.write(command)
    print("Sent drive command: Right velocity = 100 mm/s, Left velocity = 200 mm/s")

    # send 143
    #send_command(b'\x8F')  # Opcode 143 (Stop)


    # # send commands [132],[137], [255], [56], [hex8000]
    # send_command(b'\x84')  # Opcode 132 (Full Mode)
    # send_command(b'\x89')  # Opcode 137 (Drive)
    # send_command(b'\xFF')  # Velocity high byte
    # send_command(b'\x38')  # Velocity low byte
    # send_command(b'\x80')  # Radius high byte
    # send_command(b'\x00')  # Radius low byte

    # wait 1 second
    time.sleep(10)

    

    # send commands [137], [0], [0], [hex8000]
    # send_command(b'\x89')  # Opcode 137 (Drive)
    # send_command(b'\x00')  # Velocity high byte
    # send_command(b'\x00')  # Velocity low byte
    # send_command(b'\x80')  # Radius high byte
    # send_command(b'\x00')  # Radius low byte


    send_command(b'\x05')  # Length of command sequence
    send_command(b'\x91')  # Opcode 145 (Drive)
    send_command(b'\x00')  # Right velocity high byte
    send_command(b'\x00')  # Right velocity low byte
    send_command(b'\x00')  # Left velocity high byte
    send_command(b'\x00')  # Left velocity low byte (100 in decimal)
    
    get_distance()
    # request sensor data again
    #request_sensor_data()

# Execute the main function
if __name__ == '__main__':
    main()
'''
    // Print parsed data (sensor data as readable decimal values)
    cout << "Bumps and Wheeldrops:" << endl;
    cout << "  Bump Left: " << (int)bump_left << endl;
    cout << "  Bump Right: " << (int)bump_right << endl;
    cout << "  Wheel Drop Left: " << (int)wheel_drop_left << endl;
    cout << "  Wheel Drop Right: " << (int)wheel_drop_right << endl;
    cout << "  Wheel Drop Caster: " << (int)wheel_drop_caster << endl;

    cout << "Wall Sensor: " << (int)wall << endl;
    cout << "Cliff Sensors:" << endl;
    cout << "  Cliff Left: " << (int)cliff_left << endl;
    cout << "  Cliff Front Left: " << (int)cliff_front_left << endl;
    cout << "  Cliff Front Right: " << (int)cliff_front_right << endl;
    cout << "  Cliff Right: " << (int)cliff_right << endl;

    cout << "Virtual Wall: " << (int)virtual_wall << endl;

    cout << "Motor Overcurrents:" << endl;
    cout << "  Side Brush: " << (int)side_brush << endl;
    cout << "  Vacuum: " << (int)vacuum << endl;
    cout << "  Main Brush: " << (int)main_brush << endl;
    cout << "  Drive Right: " << (int)drive_right << endl;
    cout << "  Drive Left: " << (int)drive_left << endl;

    cout << "Dirt Detectors:" << endl;
    cout << "  Dirt Left: " << (int)dirt_left << endl;
    cout << "  Dirt Right: " << (int)dirt_right << endl;

    cout << "Remote Control Command: " << hex << uppercase << (int)remote_control << endl;  // Format as uppercase hex
    cout << "Buttons Pressed: " << (int)buttons << endl;

    cout << "Distance Traveled: " << (int)distance << " mm" << endl;
    cout << "Angle Turned: " << (int)angle << " mm" << endl;

    cout << "Charging State: " << charging_state_description << endl;

    cout << "Battery Information:" << endl;
    cout << "  Voltage: " << dec << voltage << " mV" << endl;
    cout << "  Current: " << dec << current << " mA" << endl;  // Correctly display signed current
    cout << "  Temperature: " << (int)temperature << " °C" << endl;
    cout << "  Charge: " << dec << charge << " mAh" << endl;  // Display charge in decimal
    cout << "  Capacity: " << dec << capacity << " mAh" << endl;  // Display capacity in decimal



'''

'''
To modify the code so that all values are stored as `double` before being printed (and to keep them consistent), you'll need to convert the necessary values into `double` and then adjust the printing accordingly. Below are the steps to achieve this:

### 1. **Convert values to `double`:**

For values that need to be displayed as `double` (like `voltage`, `current`, `charge`, `capacity`, etc.), we need to ensure that all computations or assignments before printing them treat these values as `double`.

### 2. **Update the `request_sensor_data` function:**

We will cast or convert values to `double` where applicable and ensure that everything is displayed in decimal format using `cout` and the `dec` manipulator.

### 3. **Display with appropriate formatting:**

To display values in decimal format and with proper precision, we can use `fixed` and `setprecision`.

### Here's the modified code:

```cpp
#include <iostream>
#include <fcntl.h>      // O_RDWR, O_NOCTTY, O_NONBLOCK
#include <termios.h>    // struct termios, tcgetattr, tcsetattr, baud rates
#include <unistd.h>     // write(), close(), usleep()
#include <cstring>      // memset
#include <cstdint>      // uint8_t
#include <chrono>       // std::chrono
#include <thread>       // std::this_thread::sleep_for
#include <iomanip>      // For fixed and setprecision

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
    double bumps_and_wheeldrops = response[0];
    double bump_right = bumps_and_wheeldrops & 1;  // Bit 0: Bump Right
    double bump_left = (bumps_and_wheeldrops >> 1) & 1;  // Bit 1: Bump Left
    double wheel_drop_right = (bumps_and_wheeldrops >> 2) & 1;  // Bit 2: Wheel Drop Right
    double wheel_drop_left = (bumps_and_wheeldrops >> 3) & 1;  // Bit 3: Wheel Drop Left
    double wheel_drop_caster = (bumps_and_wheeldrops >> 4) & 1;  // Bit 4: Wheel Drop Caster

    double wall = response[1];  // 1 byte, wall sensor
    double cliff_left = response[2];  // 1 byte, left cliff sensor
    double cliff_front_left = response[3];  // 1 byte, front left cliff sensor
    double cliff_front_right = response[4];  // 1 byte, front right cliff sensor
    double cliff_right = response[5];  // 1 byte, right cliff sensor
    double virtual_wall = response[6];  // 1 byte, virtual wall sensor
    double motor_overcurrents = response[7];
    double side_brush = motor_overcurrents & 1;  // Bit 0: Side Brush
    double vacuum = (motor_overcurrents >> 1) & 1;  // Bit 1: Vacuum
    double main_brush = (motor_overcurrents >> 2) & 1;  // Bit 2: Main Brush
    double drive_right = (motor_overcurrents >> 3) & 1;  // Bit 3: Drive Right
    double drive_left = (motor_overcurrents >> 4) & 1;  // Bit 4: Drive Left

    double dirt_left = response[8];  // 1 byte, dirt detector left
    double dirt_right = response[9];  // 1 byte, dirt detector right
    double remote_control = response[10];  // 1 byte, remote control command
    double buttons = response[11];  // 1 byte, button states

    // Unpack 2-byte signed distance (mm) and angle (mm)
    double distance = static_cast<double>((static_cast<int16_t>(response[12]) << 8) | response[13]);  // 2 bytes, signed distance (mm)
    double angle = static_cast<double>((static_cast<int16_t>(response[14]) << 8) | response[15]);  // 2 bytes, signed angle (mm)

    double charging_state = response[16];  // 1 byte, charging state

    // Map charging state to a description
    const char* charging_state_description = "Unknown";
    switch (charging_state) {
        case 0: charging_state_description = "Not Charging"; break;
        case 1: charging_state_description = "Charging Recovery"; break;
        case 2: charging_state_description = "Charging"; break;
        case 3: charging_state_description = "Trickle Charging"; break;
        case 4: charging_state_description = "Waiting"; break;
        case 5: charging_state_description = "Charging Error"; break;
    }

    // Unpack 2-byte unsigned values (voltage, current, charge, capacity)
    double voltage = static_cast<double>((static_cast<uint16_t>(response[17]) << 8) | response[18]);  // 2 bytes, unsigned voltage (mV)
    double current = static_cast<double>((static_cast<int16_t>(response[19]) << 8) | response[20]);  // 2 bytes, signed current (mA)
    double temperature = static_cast<double>(response[21]);  // 1 byte, temperature (°C)
    double charge = static_cast<double>((static_cast<uint16_t>(response[22]) << 8) | response[23]);  // 2 bytes, unsigned charge (mAh)
    double capacity = static_cast<double>((static_cast<uint16_t>(response[24]) << 8) | response[25]);  // 2 bytes, unsigned capacity (mAh)

    // Print parsed data (as double
```

'''

'''
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
    double current = static_cast<double>((static_cast<int16_t>(response[19]) << 8) | response[20]);  // 2 bytes, signed current (mA)
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

'''
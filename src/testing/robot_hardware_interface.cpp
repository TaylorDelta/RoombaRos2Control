// Copyright (c) 2025, TaylorDelta
// Copyright (c) 2025, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <limits>
#include <vector>

#include <fcntl.h>   // O_RDWR, O_NOCTTY, O_NONBLOCK
#include <termios.h> // struct termios, tcgetattr, tcsetattr, baud rates
#include <unistd.h>  // write(), close(), usleep()
#include <cstring>   // memset
#include <cstdint>   // uint8_t

#include "robot_hardware_interface/robot_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_hardware_interface
{
  hardware_interface::CallbackReturn RobotHardwareInterface::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    // ============================
    // on_init()
    // read and process URDF parameters
    // initalize all variables and containers
    RCLCPP_INFO(logger_, "Initializing...");
    // ============================

    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // Resize vectors to number of joints
    hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    // Resize joint limit vectors
    hw_max_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_min_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    // Resize sensor state vector
    hw_sensor_states_.resize(info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

    // Resize joint names vector
    joint_names_.resize(info_.joints.size());

    // Motor states and commands
    gpio_commands_.resize(info_.gpios.size(), 0.0);
    gpio_states_.resize(info_.gpios.size(), 0.0);   

    // Get wheelbase parameter from URDF
    wheelbase_ = std::stod(info_.hardware_parameters["wheelbase"]);

    // read min and max from command interfaces
    int idx = 0;
    for (const auto &joint : info_.joints)
    {
      for (const auto &command_interface : joint.command_interfaces)
      {
        hw_max_position_[idx] = std::stod(command_interface.max);
        hw_min_position_[idx] = std::stod(command_interface.min);
        joint_names_[idx] = joint.name;
      }
      RCLCPP_INFO(logger_, "Joint %s: velocity min=%f, max=%f", joint_names_[idx].c_str(), hw_min_position_[idx], hw_max_position_[idx]);
      idx++;
    }


    hw_commands_[0] = 0.0; // left_wheel_joint velocity
    hw_commands_[1] = 0.0; // right_wheel_joint velocity
    hw_commands_[2] = 6.0; // clean mode

    hw_states_position_[0] = 0.0; // left_wheel_joint position
    hw_states_position_[1] = 0.0; // right_wheel_joint position
    hw_states_position_[2] = 6.0; // clean mode
    hw_states_position_[3] = 0.0; // left_wheel_joint velocity
    hw_states_position_[4] = 0.0; // right_wheel_joint velocity

    hw_states_velocity_[0] = 0.0; // left_wheel_joint position  
    hw_states_velocity_[1] = 0.0; // right_wheel_joint position
    hw_states_velocity_[2] = 6.0; // clean mode
    hw_states_velocity_[3] = 0.0; // left_wheel_joint velocity
    hw_states_velocity_[4] = 0.0; // right_wheel_joint velocity

    // Initialize serial connection
    serial_fd_ = -1;

    // Initialize state variables
    last_velocity_ = 0;
    last_radius_ = 0;
    last_clean_mode_ = 6.0;

    // Odometry
    current_pose_x_ = 0.0;
    current_pose_y_ = 0.0;
    current_pose_theta_ = 0.0;

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RobotHardwareInterface::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // ============================
    // on_configure()
    // initiate comminication with the robot hardware
    // be sure which HW states can be read
    RCLCPP_INFO(logger_, "Configuring...");
    // ============================
    
    // Open serial port
    serial_fd_ = ::open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0)
    {
      RCLCPP_INFO(logger_, "Failed to open serial port /dev/ttyUSB0, trying /dev/ttyUSB1");
      serial_fd_ = ::open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NONBLOCK);
      if (serial_fd_ < 0)
      {
        RCLCPP_ERROR(logger_, "Failed to open serial port /dev/ttyUSB0 and /dev/ttyUSB1");
        RCLCPP_ERROR(logger_, "List all connected serial devices with 'ls /dev/ttyUSB*' and check permissions.");
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    // Configure serial port
    struct termios tty;
    memset(&tty, 0, sizeof tty);

    // Get current serial port settings
    if (tcgetattr(serial_fd_, &tty) != 0)
    {
      RCLCPP_ERROR(logger_, "tcgetattr() failed");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Set Baud Rate to 115200
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    // 8N1
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
    {
      RCLCPP_ERROR(logger_, "tcsetattr() failed");
    }

    // Send Roomba startup commands
    uint8_t start_cmd = 128;
    if (::write(serial_fd_, &start_cmd, 1) != 1)
    {
      RCLCPP_ERROR(logger_, "Failed to send START command (128)");
    }

    // Set SCI to FULL mode
    uint8_t full_cmd = 132;
    if (::write(serial_fd_, &full_cmd, 1) != 1)
    {
      RCLCPP_ERROR(logger_, "Failed to send FULL MODE command (132)");
    }

    
    // ---- Read Roomba Sensor Group 0 (26 bytes) ----
    uint8_t sensor_cmd[2] = {142, 0};   // 142 = Sensor, 0 = Group 0 (26 bytes)

    if (::write(serial_fd_, sensor_cmd, 2) != 2) {
        RCLCPP_ERROR(logger_, "Failed to request sensor group 0 (142,0)");
        return hardware_interface::CallbackReturn::ERROR;
    }

    usleep(10000); // give Roomba time to respond

    uint8_t response[26];
    ssize_t bytes_read = ::read(serial_fd_, response, 26);  // Use ssize_t instead of int

    RCLCPP_INFO(logger_,"Sensor Group 0 bytes read: %ld", bytes_read);

    if (bytes_read < 0) {
        RCLCPP_ERROR(logger_, "read() failed while reading sensor group 3");
        return hardware_interface::CallbackReturn::ERROR;
    }

    std::stringstream ss;
        for (int i = 0; i < bytes_read; i++) {
            ss << std::hex << std::uppercase
            << "0x" << static_cast<int>(response[i]);
        if (i < bytes_read - 1) ss << " ";
    }

    RCLCPP_INFO(logger_, "Sensor Group 0 data: %s", ss.str().c_str());


    // Parse the sensor data (convert to double where needed)
    uint8_t bumps_and_wheeldrops = response[0];
    hw_sensor_states_[0] = static_cast<double>(bumps_and_wheeldrops & 1);  // Bit 0: Bump Right
    hw_sensor_states_[1] = static_cast<double>((bumps_and_wheeldrops >> 1) & 1);  // Bit 1: Bump Left
    hw_sensor_states_[2] = static_cast<double>((bumps_and_wheeldrops >> 2) & 1);  // Bit 2: Wheel Drop Right
    hw_sensor_states_[3] = static_cast<double>((bumps_and_wheeldrops >> 3) & 1);  // Bit 3: Wheel Drop Left
    hw_sensor_states_[4] = static_cast<double>((bumps_and_wheeldrops >> 4) & 1);  // Bit 4: Wheel Drop Caster
    hw_sensor_states_[5] = static_cast<double>(response[1]);  // 1 byte, wall sensor
    hw_sensor_states_[6] = static_cast<double>(response[2]);  // 1 byte, left cliff sensor
    hw_sensor_states_[7] = static_cast<double>(response[3]);  // 1 byte, front left cliff sensor
    hw_sensor_states_[8] = static_cast<double>(response[4]);  // 1 byte, front right cliff sensor
    hw_sensor_states_[9] = static_cast<double>(response[5]);  // 1 byte, right cliff sensor
    hw_sensor_states_[10] = static_cast<double>(response[6]);  // 1 byte, virtual wall sensor
    uint8_t motor_overcurrents = response[7];
    hw_sensor_states_[11] = static_cast<double>(motor_overcurrents & 1);  // Bit 0: Side Brush
    hw_sensor_states_[12] = static_cast<double>((motor_overcurrents >> 1) & 1);  // Bit 1: Vacuum
    hw_sensor_states_[13] = static_cast<double>((motor_overcurrents >> 2) & 1);  // Bit 2: Main Brush
    hw_sensor_states_[14] = static_cast<double>((motor_overcurrents >> 3) & 1);  // Bit 3: Drive Right
    hw_sensor_states_[15] = static_cast<double>((motor_overcurrents >> 4) & 1);  // Bit 4: Drive Left
    hw_sensor_states_[16] = static_cast<double>(response[8]);  // 1 byte, dirt detector left
    hw_sensor_states_[17] = static_cast<double>(response[9]);  // 1 byte, dirt detector right
    hw_sensor_states_[18] = static_cast<double>(response[10]);  // 1 byte, remote control command
    uint8_t buttons = response[11];
    hw_sensor_states_[19] = static_cast<double>(buttons & 1);        // Bit 0: Max
    hw_sensor_states_[20] = static_cast<double>((buttons >> 1) & 1); // Bit 1: Clean
    hw_sensor_states_[21] = static_cast<double>((buttons >> 2) & 1); // Bit 2: Spot 
    hw_sensor_states_[22] = static_cast<double>((buttons >> 3) & 1); // Bit 3: Power
    distance_ = static_cast<double>((static_cast<int16_t>(response[12] << 8 | response[13])));  // 2 bytes, signed distance (mm)
    angle_ = static_cast<double>((static_cast<int16_t>(response[14] << 8 | response[15])));  // 2 bytes, signed angle (mm)
    hw_sensor_states_[23] = static_cast<double>(response[16]);  // 1 byte, charging state
    hw_sensor_states_[24] = static_cast<double>((static_cast<uint16_t>(response[17]) << 8) | response[18]);  // 2 bytes, unsigned voltage (mV)
    hw_sensor_states_[25] = static_cast<double>((static_cast<int16_t>(response[19] << 8) | response[20]));  // 2 bytes, signed current (mA)
    hw_sensor_states_[26] = static_cast<double>(response[21]);  // 1 byte, temperature (°C)
    hw_sensor_states_[27] = static_cast<double>((static_cast<uint16_t>(response[22]) << 8) | response[23]);  // 2 bytes, unsigned charge (mAh)
    hw_sensor_states_[28] = static_cast<double>((static_cast<uint16_t>(response[24]) << 8) | response[25]);  // 2 bytes, unsigned capacity (mAh)

    RCLCPP_INFO(logger_, "Bumps and Wheeldrops:");
    RCLCPP_INFO(logger_, "  Bump Left: %f", hw_sensor_states_[1]);
    RCLCPP_INFO(logger_, "  Bump Right: %f", hw_sensor_states_[2]);
    RCLCPP_INFO(logger_, "  Wheel Drop Left: %f", hw_sensor_states_[3]);
    RCLCPP_INFO(logger_, "  Wheel Drop Right: %f", hw_sensor_states_[4]);
    RCLCPP_INFO(logger_, "  Wheel Drop Caster: %f", hw_sensor_states_[5]);
    RCLCPP_INFO(logger_, "Wall Sensor: %f", hw_sensor_states_[6]);
    RCLCPP_INFO(logger_, "Cliff Sensors:");
    RCLCPP_INFO(logger_, "  Cliff Left: %f", hw_sensor_states_[7]);
    RCLCPP_INFO(logger_, "  Cliff Front Left: %f", hw_sensor_states_[8]);
    RCLCPP_INFO(logger_, "  Cliff Front Right: %f", hw_sensor_states_[9]);
    RCLCPP_INFO(logger_, "  Cliff Right: %f", hw_sensor_states_[10]);
    RCLCPP_INFO(logger_, "Virtual Wall: %f", hw_sensor_states_[11]);
    RCLCPP_INFO(logger_, "Motor Overcurrents:");
    RCLCPP_INFO(logger_, "  Side Brush: %f", hw_sensor_states_[12]);
    RCLCPP_INFO(logger_, "  Vacuum: %f", hw_sensor_states_[13]);
    RCLCPP_INFO(logger_, "  Main Brush: %f", hw_sensor_states_[14]);
    RCLCPP_INFO(logger_, "  Drive Right: %f", hw_sensor_states_[15]);
    RCLCPP_INFO(logger_, "  Drive Left: %f", hw_sensor_states_[16]);
    RCLCPP_INFO(logger_, "Dirt Detectors:");
    RCLCPP_INFO(logger_, "  Dirt Left: %f", hw_sensor_states_[17]);
    RCLCPP_INFO(logger_, "  Dirt Right: %f", hw_sensor_states_[18]);
    RCLCPP_INFO(logger_, "Remote Control Command: %f", hw_sensor_states_[19]);
    RCLCPP_INFO(logger_, "Button Max: %f", hw_sensor_states_[20]);
    RCLCPP_INFO(logger_, "Button Clean: %f", hw_sensor_states_[21]);
    RCLCPP_INFO(logger_, "Button Spot: %f", hw_sensor_states_[22]);
    RCLCPP_INFO(logger_, "Button Power: %f", hw_sensor_states_[23]);
    RCLCPP_INFO(logger_, "Distance Traveled: %f meters", distance_);
    RCLCPP_INFO(logger_, "Angle Turned: %f", angle_);
    RCLCPP_INFO(logger_, "Charging State: %f", hw_sensor_states_[21]);
    RCLCPP_INFO(logger_, "Battery Information:");
    RCLCPP_INFO(logger_, "  Voltage: %f V", hw_sensor_states_[24]);
    RCLCPP_INFO(logger_, "  Current: %f A", hw_sensor_states_[25]);
    RCLCPP_INFO(logger_, "  Temperature: %f °C", hw_sensor_states_[26]);
    RCLCPP_INFO(logger_, "  Charge: %f mAh", hw_sensor_states_[27]);
    RCLCPP_INFO(logger_, "  Capacity: %f mAh", hw_sensor_states_[28]);
    RCLCPP_INFO(logger_, "Roomba placed in FULL mode");
    
   
    // // ----------
    // // Ask sensor data to empty buffers
    // uint8_t sensor_cmd[2] = {142, 2}; // 142 = Sensor, 1 = Group 1 (10 bytes)

    // if (::write(serial_fd_, sensor_cmd, 2) != 2)
    // {
    //   RCLCPP_INFO(logger_, "Empty Buffer (142,2)");
    //   // return hardware_interface::return_type::ERROR;
    // }

    uint8_t sensor_cmd_149[4] = {149, 2, 43, 44}; // 149 = Sensor, 2 = Group 2 (6 bytes)

    if (::write(serial_fd_, sensor_cmd_149, 4) != 4)
    {
      RCLCPP_ERROR(logger_, "Failed to request sensor group 0 (142,0)");
      // return hardware_interface::return_type::ERROR;
    }

    uint8_t response_2[4];
    ssize_t bytes_read_2 = ::read(serial_fd_, response_2, 4);  // Use ssize_t instead of int

    uint16_t left_counts_ = static_cast<int16_t>(response_2[0] << 8 | response_2[1]);
    uint16_t right_counts_ = static_cast<int16_t>(response_2[2] << 8 | response_2[3]);

    RCLCPP_INFO(logger_, "Initial Left Wheel Encoder Counts: %f", left_counts_);
    RCLCPP_INFO(logger_, "Initial Right Wheel Encoder Counts: %f", right_counts_);
    last_left_encoder_counts_read = left_counts_;
    last_right_encoder_counts_read = right_counts_;


    // ----------

    // Odometry
    current_pose_x_ = 0.0;
    current_pose_y_ = 0.0;
    current_pose_theta_ = 0.0;

    hw_states_position_[0] = current_pose_x_; // left_wheel_joint position
    hw_states_position_[1] = current_pose_y_; // right_wheel_joint position

    // Initialize state variables
    last_velocity_ = 0;
    last_radius_ = 0;
    last_clean_mode_ = 6.0;

    RCLCPP_INFO(logger_, "Configuration successful.");

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> RobotHardwareInterface::export_state_interfaces()
  {
    // ============================
    // export_state_interfaces()
    // return list of state interfaces
    RCLCPP_INFO(logger_, "Exporting state interfaces...");
    // ============================

    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
    }

    // export sensor state interface
    for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
    {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_[i]));
    }

    // Export GPIO state interfaces
    for (size_t i = 0; i < info_.gpios.size(); ++i)
    { 
      for (size_t j = 0; j < info_.gpios[i].state_interfaces.size(); ++j)
      {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.gpios[i].name,   // GPIO name
                info_.gpios[i].state_interfaces[j].name,               // Interface type for GPIO
                &gpio_states_[i]));    // Pointer to the GPIO state variable
      }
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> RobotHardwareInterface::export_command_interfaces()
  {
    // ============================
    // export_command_interfaces()
    // return list of command interfaces
    RCLCPP_INFO(logger_, "Exporting command interfaces...");
    // ============================
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }

    // Export GPIO command interfaces
    for (size_t i = 0; i < info_.gpios.size(); ++i)
    { 
      for (size_t j = 0; j < info_.gpios[i].command_interfaces.size(); ++j)
      {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.gpios[i].name,   // GPIO name
                info_.gpios[i].command_interfaces[j].name,               // Interface type for GPIO
                &gpio_commands_[i]));    // Pointer to the GPIO command variable
      }
    }


    return command_interfaces;
  }

  hardware_interface::CallbackReturn RobotHardwareInterface::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // ============================
    // on_activate()
    // prepare the robot to receive commands
    RCLCPP_INFO(logger_, "Activated...");
    // ============================

    uint8_t sensor_cmd_149[4] = {149, 2, 43, 44}; // 149 = Sensor, 2 = Group 2 (6 bytes)

    if (::write(serial_fd_, sensor_cmd_149, 4) != 4)
    {
      RCLCPP_ERROR(logger_, "Failed to request sensor group 0 (142,0)");
      // return hardware_interface::return_type::ERROR;
    }

    uint8_t response_2[4];
    ssize_t bytes_read_2 = ::read(serial_fd_, response_2, 4);  // Use ssize_t instead of int

    uint16_t left_counts_ = static_cast<int16_t>(response_2[0] << 8 | response_2[1]);
    uint16_t right_counts_ = static_cast<int16_t>(response_2[2] << 8 | response_2[3]);

    RCLCPP_INFO(logger_, "Initial Left Wheel Encoder Counts: %f", left_counts_);
    RCLCPP_INFO(logger_, "Initial Right Wheel Encoder Counts: %f", right_counts_);
    last_left_encoder_counts_read = left_counts_;
    last_right_encoder_counts_read = right_counts_;

    // ----------

    // Odometry
    current_pose_x_ = 0.0;
    current_pose_y_ = 0.0;
    current_pose_theta_ = 0.0;

    hw_states_position_[0] = current_pose_x_; // left_wheel_joint position
    hw_states_position_[1] = current_pose_y_; // right_wheel_joint position
    hw_states_position_[2] = 6.0; // clean mode
    hw_states_position_[3] = current_pose_x_; // left_wheel_joint velocity
    hw_states_position_[4] = current_pose_y_; // right_wheel_joint velocity

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RobotHardwareInterface::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Send DRIVE command to stop the robot
    int16_t velocity = 0; // mm/s
    int16_t radius = 0;   // straight

    uint8_t drive_cmd[5] = {
        137,
        static_cast<uint8_t>((velocity >> 8) & 0xFF),
        static_cast<uint8_t>(velocity & 0xFF),
        static_cast<uint8_t>((radius >> 8) & 0xFF),
        static_cast<uint8_t>(radius & 0xFF)};

    if (::write(serial_fd_, drive_cmd, 5) != 5)
    {
      RCLCPP_ERROR(logger_, "Failed to send DRIVE command");
    }
    else
    {
      RCLCPP_INFO(logger_, "Sent STOP command to robot.");
    }

    // Set SCI to PASSIVE mode
    uint8_t passive_mode = 128;
    if (::write(serial_fd_, &passive_mode, 1) != 1)
    {
      RCLCPP_ERROR(logger_, "Failed to send Start command (128)");
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(logger_, "Deactivated.");

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RobotHardwareInterface::on_cleanup(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Close serial port
    if (serial_fd_ >= 0)
    {
      ::close(serial_fd_);
      serial_fd_ = -1;
      RCLCPP_INFO(logger_, "Serial port closed.");
    }

    RCLCPP_INFO(logger_, "Cleaned up.");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RobotHardwareInterface::on_shutdown(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Close serial port
    if (serial_fd_ >= 0)
    {
      ::close(serial_fd_);
      serial_fd_ = -1;
      RCLCPP_INFO(logger_, "Serial port closed.");
    }

    RCLCPP_INFO(logger_, "Shutdown complete.");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type RobotHardwareInterface::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  { 
    // ============================
    // read()
    // read the current state from the robot hardware
    // ============================

    // read encoder values from Roomba
    // packages 43 and 44
    // Serial sequence: [149][Number of Packets][Packet ID 1][Packet ID 2]...[Packet ID N]
    // send 149 2 43 44

    uint8_t sensor_cmd[4] = {149, 2, 43, 44}; // 149 = Sensor, 2 = Group 2 (6 bytes)

    if (::write(serial_fd_, sensor_cmd, 4) != 4)
    {
      RCLCPP_ERROR(logger_, "Failed to request sensor group 0 (142,0)");
      // return hardware_interface::return_type::ERROR;
    }

    uint8_t response[4];
    ssize_t bytes_read = ::read(serial_fd_, response, 4);  // Use ssize_t instead of int

    // Read encoder counts from the response
    uint16_t left_encoder_counts_read = static_cast<uint16_t>(response[0] << 8 | response[1]);
    uint16_t right_encoder_counts_read = static_cast<uint16_t>(response[2] << 8 | response[3]);

    // Normalize previous encoder counts using wrap-around logic (16-bit encoder)
    double previous_left_encoder_counts_normalized = static_cast<double>(last_left_encoder_counts_read & 0xFFFF);
    double previous_right_encoder_counts_normalized = static_cast<double>(last_right_encoder_counts_read & 0xFFFF);

    last_left_encoder_counts_read = left_encoder_counts_read;
    last_right_encoder_counts_read = right_encoder_counts_read;

    // Cast encoder counts to signed 16-bit for proper delta calculation
    double left_encoder_counts_ = static_cast<int16_t>(left_encoder_counts_read);
    double right_encoder_counts_ = static_cast<int16_t>(right_encoder_counts_read);

    // Calculate delta (change in encoder counts) for both wheels
    double delta_left = 0.0;
    double delta_right = 0.0;

    // Handle left encoder wrap-around
    if (left_encoder_counts_ >= previous_left_encoder_counts_normalized) {
        delta_left = left_encoder_counts_ - previous_left_encoder_counts_normalized;
    } else {
        delta_left = (65536.0 - previous_left_encoder_counts_normalized) + left_encoder_counts_;
    }

    // Handle right encoder wrap-around
    if (right_encoder_counts_ >= previous_right_encoder_counts_normalized) {
        delta_right = right_encoder_counts_ - previous_right_encoder_counts_normalized;
    } else {
        delta_right = (65536.0 - previous_right_encoder_counts_normalized) + right_encoder_counts_;
    }

    // Accumulate the deltas (make sure these variables are declared and initialized somewhere)
    left_wheel_encoder_counts_ += delta_left;
    right_wheel_encoder_counts_ += delta_right;


    double tick_to_distance_ = (72 * 3.14159) / 508.8;  // in mm
    double left_distance = delta_left * tick_to_distance_;
    double right_distance = delta_right * tick_to_distance_;  // in mm

    distance_ = (left_distance + right_distance) / 2.0;
    angle_ = (right_distance - left_distance)/ 2.0;

    // Update odometry
    current_pose_theta_ += (2 * angle_) / (wheelbase_ * 1000); // * 3.14159; // times pi, there is an error
    current_pose_x_ -= distance_ * cos(current_pose_theta_); // use negative distance to match coordinate frame
    current_pose_y_ -= distance_ * sin(current_pose_theta_); //

    // write pose state interfaces
    hw_states_position_[0] = current_pose_x_; // left_wheel_joint position
    hw_states_position_[1] = current_pose_y_; // right_wheel_joint position


    // for robot_simple_controller, listens to joint 3 and 4 for odometry
    hw_states_position_[3] = current_pose_x_; // left_wheel_joint position
    hw_states_position_[4] = current_pose_y_; // right_wheel_joint position
    //hw_states_position_[5] = 0.0f; // right_wheel_joint position

    hw_states_velocity_[3] = current_pose_x_; // left_wheel_joint position
    hw_states_velocity_[4] = current_pose_y_; // right_wheel_joint position
    //hw_states_velocity_[5] = 0.0f; // right_wheel_joint position

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type RobotHardwareInterface::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  { 
    // ============================
    // write()
    // write the current command to the robot hardware
    // ============================

    // Only if clean_mode_ is a new value, send the appropriate command
    // Modes: Power 133, Spot 134, Clean 135, Max 136

    /*// works with ubuntu@ubuntu:~$ ros2 topic pub --once /robot/clean_mode std_msgs/msg/Int32 "{data: 2}"
    publisher: beginning loop
    publishing #1: std_msgs.msg.Int32(data=2)
    but does not send the command yet
   */
    // clean_mode_ = hw_commands_[2]; // assuming the 3rd command interface is for clean_mode
    // if (clean_mode_ != last_clean_mode_) {
    //   uint8_t clean_cmd = 0;
    //   switch (static_cast<int>(clean_mode_)) {
    //     case 130:
    //       clean_cmd = 130; // Control
    //       break;
    //     case 131:
    //       clean_cmd = 131; // Safe
    //       break;
    //     case 132:
    //       clean_cmd = 132; // Full
    //       break;
    //     case 133:
    //       clean_cmd = 133; // Power
    //       break;
    //     case 134:
    //       clean_cmd = 134; // Spot
    //       break;
    //     case 135:
    //       clean_cmd = 135; // Clean
    //       break;
    //     case 136:
    //       clean_cmd = 136; // Max
    //       break;

    //     default:
    //       RCLCPP_WARN(logger_, "Invalid clean mode: %f", clean_mode_);
    //       clean_cmd = 0;
    //       break;
    //   }
    //   if (clean_cmd != 0) {

    //     uint8_t full_cmd = 132;
    //     if (::write(serial_fd_, &full_cmd, 1) != 1) {
    //         RCLCPP_ERROR(logger_, "Failed to send FULL MODE command (132)");
    //         return hardware_interface::return_type::ERROR;
    //     }
    //     usleep(10000); // give Roomba time to respond
    //     if (::write(serial_fd_, &clean_cmd, 1) != 1) {
    //       RCLCPP_ERROR(logger_, "Failed to send CLEAN MODE command: %d", clean_cmd);
    //       return hardware_interface::return_type::ERROR;
    //     } else {
    //       RCLCPP_INFO(logger_, "Sent CLEAN MODE command: %d", clean_cmd);
    //       last_clean_mode_ = clean_mode_;
    //     }
    //   }
    // }

    // ======================================
    // Send DRIVE command
    // ======================================

    double linear_velocity_ = (hw_commands_[1] + hw_commands_[0]) / 2.0 * 1000;                               // calculate velocity
    double radius_ = ((hw_commands_[1] + hw_commands_[0]) / 2.0) * wheelbase_*1000 / (hw_commands_[1] - hw_commands_[0]); // calculate radius

    // Convert to int16_t
    int16_t velocity = static_cast<int16_t>(linear_velocity_); // mm/s
    int16_t radius = static_cast<int16_t>(radius_);            // Control for turning radius

    if (velocity != last_velocity_ || radius != last_radius_)
    {
      RCLCPP_INFO(logger_, "Velocity command: %d mm/s, Radius command: %d mm\n", velocity, radius);
      RCLCPP_INFO(logger_, "HW Commands: 3: %f , 4: %f\n", hw_commands_[0], hw_commands_[1]);
      // Check for velocity limits
      if (velocity > 400)
        velocity = 400;
      if (velocity < -400)
        velocity = -400;

      uint8_t drive_cmd[5]; // Declare the drive_cmd array before the if-else block

      if (radius > 2000 || radius < -2000)
      {
        // Set radius to hex8000 if it's out of bounds
        drive_cmd[0] = 137;                                          // Command identifier for DRIVE
        drive_cmd[1] = static_cast<uint8_t>((velocity >> 8) & 0xFF); // High byte of velocity
        drive_cmd[2] = static_cast<uint8_t>(velocity & 0xFF);        // Low byte of velocity
        drive_cmd[3] = 0x80;                                         // High byte of radius
        drive_cmd[4] = 0x00;                                         // Low byte of radius
      }
      else
      {
        // Use actual radius if it's within bounds
        drive_cmd[0] = 137;                                          // Command identifier for DRIVE
        drive_cmd[1] = static_cast<uint8_t>((velocity >> 8) & 0xFF); // High byte of velocity
        drive_cmd[2] = static_cast<uint8_t>(velocity & 0xFF);        // Low byte of velocity
        drive_cmd[3] = static_cast<uint8_t>((radius >> 8) & 0xFF);   // High byte of radius
        drive_cmd[4] = static_cast<uint8_t>(radius & 0xFF);          // Low byte of radius
      }

      // Send the command via serial
      if (::write(serial_fd_, drive_cmd, 5) != 5)
      {
        RCLCPP_ERROR(logger_, "Failed to send DRIVE command");
        return hardware_interface::return_type::ERROR;
      }
      else
      {
        RCLCPP_INFO(logger_, "Sent DRIVE command: vel=%d mm/s, radius=%d mm",
                    velocity, radius);
      }

      // Update last sent values
      last_velocity_ = velocity;
      last_radius_ = radius;
    }

    return hardware_interface::return_type::OK;
  }

} // namespace robot_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    robot_hardware_interface::RobotHardwareInterface, hardware_interface::SystemInterface)

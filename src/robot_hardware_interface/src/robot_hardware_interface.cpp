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

#include <fcntl.h>      // O_RDWR, O_NOCTTY, O_NONBLOCK
#include <termios.h>    // struct termios, tcgetattr, tcsetattr, baud rates
#include <unistd.h>     // write(), close(), usleep()
#include <cstring>      // memset
#include <cstdint>      // uint8_t

#include "robot_hardware_interface/robot_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_hardware_interface
{
hardware_interface::CallbackReturn RobotHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // TODO(anyone): read parameters and initialize the hardware
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(logger_, "HardwareInterface on_init");
  serial_fd_ = -1;
  last_velocity_ = 0;
  last_radius_ = 0;

  linear_velocity_ = 0.0;
  angular_velocity_ = 0.0;


  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to be ready for read calls and write calls of some interfaces
  RCLCPP_INFO(logger_, "Configuring Hardware interface...");

  last_velocity_ = 0;
  last_radius_ = 0;

  linear_velocity_ = 0.0;
  angular_velocity_ = 0.0;

  // Open serial port
  serial_fd_ = ::open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
      RCLCPP_INFO(logger_, "Failed to open serial port /dev/ttyUSB0, trying /dev/ttyUSB1");
      serial_fd_ = ::open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NONBLOCK);
      if (serial_fd_ < 0) {
          RCLCPP_ERROR(logger_, "Failed to open serial port /dev/ttyUSB0 and /dev/ttyUSB1");
          RCLCPP_ERROR(logger_, "List all connected serial devices with 'ls /dev/ttyUSB*' and check permissions.");
          return hardware_interface::CallbackReturn::ERROR;
      }
  }

  struct termios tty;
  memset(&tty, 0, sizeof tty);

  if (tcgetattr(serial_fd_, &tty) != 0) {
      RCLCPP_ERROR(logger_, "tcgetattr() failed");
      return hardware_interface::CallbackReturn::ERROR;
  }

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

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(logger_, "tcsetattr() failed");
      return hardware_interface::CallbackReturn::ERROR;
  }

  // Send Roomba startup commands
  uint8_t start_cmd = 128;
  if (::write(serial_fd_, &start_cmd, 1) != 1) {
      RCLCPP_ERROR(logger_, "Failed to send START command (128)");
      return hardware_interface::CallbackReturn::ERROR;
  }
  usleep(20000);  // 20 ms delay

  uint8_t full_cmd = 132;
  if (::write(serial_fd_, &full_cmd, 1) != 1) {
      RCLCPP_ERROR(logger_, "Failed to send FULL MODE command (132)");
      return hardware_interface::CallbackReturn::ERROR;
  }


  // ---- Read Roomba Sensor Group 3 (10 bytes) ----
  uint8_t sensor_cmd[2] = {142, 3};   // 142 = Sensor, 3 = Group 3 (10 bytes)

  if (::write(serial_fd_, sensor_cmd, 2) != 2) {
      RCLCPP_ERROR(logger_, "Failed to request sensor group 3 (142,3)");
      return hardware_interface::CallbackReturn::ERROR;
  }

  usleep(20000); // give Roomba time to respond

  uint8_t sensor_buf[10];
  ssize_t bytes_read = ::read(serial_fd_, sensor_buf, 10);  // Use ssize_t instead of int

  if (bytes_read < 0) { 
      RCLCPP_ERROR(logger_, "read() failed while reading sensor group 3");
      return hardware_interface::CallbackReturn::ERROR;
  }


  RCLCPP_INFO(logger_,
          "Sensor Group 3 bytes read: %ld", bytes_read);

  std::stringstream ss;
      for (int i = 0; i < bytes_read; i++) {
          ss << std::hex << std::uppercase
          << "0x" << static_cast<int>(sensor_buf[i]);
      if (i < bytes_read - 1) ss << " ";
  }

  RCLCPP_INFO(logger_, "Sensor Group 3 data: %s", ss.str().c_str());

  RCLCPP_INFO(logger_, "Roomba placed in FULL mode");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Define the state interfaces for linear velocity and angular velocity
    state_interfaces.push_back(hardware_interface::StateInterface("linear_velocity_joint", "position", &linear_position_));
    state_interfaces.push_back(hardware_interface::StateInterface("linear_velocity_joint", "velocity", &linear_velocity_));
    state_interfaces.push_back(hardware_interface::StateInterface("linear_velocity_joint", "acceleration", &linear_acceleration_));
    state_interfaces.push_back(hardware_interface::StateInterface("angular_velocity_joint", "position", &angular_position_));
    state_interfaces.push_back(hardware_interface::StateInterface("angular_velocity_joint", "velocity", &angular_velocity_));
    state_interfaces.push_back(hardware_interface::StateInterface("angular_velocity_joint", "acceleration", &angular_acceleration_));

    // State interfaces for Sensor Data
    state_interfaces.push_back(hardware_interface::StateInterface("state", "bump_right", &bump_right_));
    state_interfaces.push_back(hardware_interface::StateInterface("state", "bump_left", &bump_left_));

    state_interfaces.push_back(hardware_interface::StateInterface("state", "wheeldrops_right", &wheeldrops_right_));
    state_interfaces.push_back(hardware_interface::StateInterface("state", "wheeldrops_left", &wheeldrops_left_));
    state_interfaces.push_back(hardware_interface::StateInterface("state", "wheeldrops_caster", &wheeldrops_caster_));

    state_interfaces.push_back(hardware_interface::StateInterface("state", "wall", &wall_));  // No "wall_left" or "wall_right"
    state_interfaces.push_back(hardware_interface::StateInterface("state", "cliff_left", &cliff_left_));
    state_interfaces.push_back(hardware_interface::StateInterface("state", "cliff_front_left", &cliff_front_left_));
    state_interfaces.push_back(hardware_interface::StateInterface("state", "cliff_front_right", &cliff_front_right_));
    state_interfaces.push_back(hardware_interface::StateInterface("state", "cliff_right", &cliff_right_));
    state_interfaces.push_back(hardware_interface::StateInterface("state", "virtual_wall", &virtual_wall_));

    state_interfaces.push_back(hardware_interface::StateInterface("state", "motor_overcurrents_sidebrush", &motor_overcurrents_sidebrush_));
    state_interfaces.push_back(hardware_interface::StateInterface("state", "motor_overcurrents_vacuum", &motor_overcurrents_vacuum_));
    state_interfaces.push_back(hardware_interface::StateInterface("state", "motor_overcurrents_mainbrush", &motor_overcurrents_mainbrush_));
    state_interfaces.push_back(hardware_interface::StateInterface("state", "motor_overcurrents_driveright", &motor_overcurrents_driveright_));
    state_interfaces.push_back(hardware_interface::StateInterface("state", "motor_overcurrents_driveleft", &motor_overcurrents_driveleft_));

    state_interfaces.push_back(hardware_interface::StateInterface("state", "dirt_detector_left", &dirt_detector_left_));
    state_interfaces.push_back(hardware_interface::StateInterface("state", "dirt_detector_right", &dirt_detector_right_));

    state_interfaces.push_back(hardware_interface::StateInterface("state", "remote_opcode", &remote_opcode_));

    state_interfaces.push_back(hardware_interface::StateInterface("state", "buttons", &buttons_));

    state_interfaces.push_back(hardware_interface::StateInterface("state", "distance", &distance_));
    state_interfaces.push_back(hardware_interface::StateInterface("state", "angle", &angle_));

    state_interfaces.push_back(hardware_interface::StateInterface("state", "charging_state", &charging_state_));

    state_interfaces.push_back(hardware_interface::StateInterface("state", "voltage", &voltage_));
    state_interfaces.push_back(hardware_interface::StateInterface("state", "current", &current_));

    state_interfaces.push_back(hardware_interface::StateInterface("state", "temperature", &temperature_));

    state_interfaces.push_back(hardware_interface::StateInterface("state", "charge", &charge_));
    state_interfaces.push_back(hardware_interface::StateInterface("state", "capacity", &capacity_));


    return state_interfaces;
}




std::vector<hardware_interface::CommandInterface> RobotHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Define the command interfaces for linear velocity and angular velocity
  command_interfaces.push_back(hardware_interface::CommandInterface("linear_velocity_joint", "velocity", &linear_velocity_));
  command_interfaces.push_back(hardware_interface::CommandInterface("angular_velocity_joint", "velocity", &angular_velocity_));
  command_interfaces.push_back(hardware_interface::CommandInterface("linear_velocity_joint", "position", &linear_position_));
  command_interfaces.push_back(hardware_interface::CommandInterface("angular_velocity_joint", "position", &angular_position_));

  return command_interfaces;
}

hardware_interface::CallbackReturn RobotHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to receive commands
  RCLCPP_INFO(logger_, "Hardware interface activated.");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  
  // TODO(anyone): prepare the robot to stop receiving commands
  int16_t velocity = 0;      // mm/s
  int16_t radius   = 0;   // straight

  uint8_t drive_cmd[5] = {
      137,
      static_cast<uint8_t>((velocity >> 8) & 0xFF),
      static_cast<uint8_t>(velocity & 0xFF),
      static_cast<uint8_t>((radius >> 8) & 0xFF),
      static_cast<uint8_t>(radius & 0xFF)
  };

  if (::write(serial_fd_, drive_cmd, 5) != 5) {
      RCLCPP_ERROR(logger_, "Failed to send DRIVE command");
  } else {
      RCLCPP_INFO(logger_, "Sent DRIVE command: vel=%d mm/s, radius=0x%04X",
                  velocity, radius);
  }

  uint8_t full_cmd = 128;
  if (::write(serial_fd_, &full_cmd, 1) != 1) {
      RCLCPP_ERROR(logger_, "Failed to send Start command (128)");
      return hardware_interface::CallbackReturn::ERROR;
  }
  
  RCLCPP_INFO(logger_, "Hardware interface deactivated.");
  
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(anyone): read robot states

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    /*
    Serial sequence: [137] [Velocity high byte] [Velocity low byte]
    [Radius high byte] [Radius low byte]
    Drive data bytes 1 and 2: Velocity (-500 – 500 mm/s)
    Drive data bytes 3 and 4: Radius (-2000 – 2000 mm)
    */
    
    // Check if serial port is open
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(logger_, "Serial port not open");
        return hardware_interface::return_type::ERROR;
    }

    // Get the current velocities from the command interfaces 
    int16_t velocity = static_cast<int16_t>(linear_velocity_);  // mm/s
    int16_t radius = static_cast<int16_t>(angular_velocity_);  // Control for turning radius

    // Check for velocity limits
    if (velocity > 500) velocity = 500;
    if (velocity < -500) velocity = -500;
    
    // Check for radius limits
    if (radius > 2000) radius = 2000;
    if (radius < -2000) radius = -2000;

    // Only send data if it differs from the last sent values
    if (velocity == last_velocity_ && radius == last_radius_) {
        return hardware_interface::return_type::OK;
    }

    // Create a byte array for the command (5 bytes for the drive command)
    uint8_t drive_cmd[5] = {
        137,  // Command identifier (example value, needs to match your device protocol)
        static_cast<uint8_t>((velocity >> 8) & 0xFF), // High byte of velocity
        static_cast<uint8_t>(velocity & 0xFF),        // Low byte of velocity
        static_cast<uint8_t>((radius >> 8) & 0xFF),   // High byte of radius
        static_cast<uint8_t>(radius & 0xFF)           // Low byte of radius
    };

    // Send the command to the hardware over the serial interface
    if (::write(serial_fd_, drive_cmd, sizeof(drive_cmd)) != sizeof(drive_cmd)) {
        RCLCPP_ERROR(logger_, "Failed to send DRIVE command");
        return hardware_interface::return_type::ERROR;
    } else {
        RCLCPP_INFO(logger_, "Sent DRIVE command: vel=%d mm/s, radius=0x%04X", velocity, radius);
        last_velocity_ = velocity;
        last_radius_ = radius;
    }

    // ---- Test communication (Read all 26 bytes for Group 0) ----
    // Request sensor data (Group 0: all sensor states)
    uint8_t sensor_cmd[2] = {142, 0};   // 142 = Sensor, 3 = Group 3 (10 bytes)

    if (::write(serial_fd_, sensor_cmd, 2) != 2) {
        RCLCPP_ERROR(logger_, "Failed to request sensor group 0 (142,0)");
        return hardware_interface::return_type::ERROR;
    }

    usleep(20000); // give Roomba time to respond

    uint8_t response[26];
    ssize_t bytes_read = ::read(serial_fd_, response, 26);  // Use ssize_t instead of int

    RCLCPP_INFO(logger_,"Sensor Group 0 bytes read: %ld", bytes_read);

    if (bytes_read < 0) { 
        RCLCPP_ERROR(logger_, "read() failed while reading sensor group 3");
        return hardware_interface::return_type::ERROR;
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
    bump_right_ = static_cast<double>(bumps_and_wheeldrops & 1);  // Bit 0: Bump Right
    bump_left_ = static_cast<double>((bumps_and_wheeldrops >> 1) & 1);  // Bit 1: Bump Left
    wheeldrops_right_ = static_cast<double>((bumps_and_wheeldrops >> 2) & 1);  // Bit 2: Wheel Drop Right
    wheeldrops_left_ = static_cast<double>((bumps_and_wheeldrops >> 3) & 1);  // Bit 3: Wheel Drop Left
    wheeldrops_caster_ = static_cast<double>((bumps_and_wheeldrops >> 4) & 1);  // Bit 4: Wheel Drop Caster

    wall_ = static_cast<double>(response[1]);  // 1 byte, wall sensor
    cliff_left_ = static_cast<double>(response[2]);  // 1 byte, left cliff sensor
    cliff_front_left_ = static_cast<double>(response[3]);  // 1 byte, front left cliff sensor
    cliff_front_right_ = static_cast<double>(response[4]);  // 1 byte, front right cliff sensor
    cliff_right_ = static_cast<double>(response[5]);  // 1 byte, right cliff sensor
    virtual_wall_ = static_cast<double>(response[6]);  // 1 byte, virtual wall sensor
    uint8_t motor_overcurrents = response[7];
    motor_overcurrents_sidebrush_ = static_cast<double>(motor_overcurrents & 1);  // Bit 0: Side Brush
    motor_overcurrents_vacuum_ = static_cast<double>((motor_overcurrents >> 1) & 1);  // Bit 1: Vacuum
    motor_overcurrents_mainbrush_ = static_cast<double>((motor_overcurrents >> 2) & 1);  // Bit 2: Main Brush
    motor_overcurrents_driveright_ = static_cast<double>((motor_overcurrents >> 3) & 1);  // Bit 3: Drive Right
    motor_overcurrents_driveleft_ = static_cast<double>((motor_overcurrents >> 4) & 1);  // Bit 4: Drive Left

    dirt_detector_left_ = static_cast<double>(response[8]);  // 1 byte, dirt detector left
    dirt_detector_right_ = static_cast<double>(response[9]);  // 1 byte, dirt detector right
    remote_opcode_ = static_cast<double>(response[10]);  // 1 byte, remote control command
    buttons_ = static_cast<double>(response[11]);  // 1 byte, button states

    // Unpack 2-byte signed distance (mm) and angle (mm)
    distance_ = static_cast<double>((static_cast<int16_t>(response[12]) << 8 | response[13]));  // 2 bytes, signed distance (mm)
    angle_ = static_cast<double>((static_cast<int16_t>(response[14]) << 8 | response[15]));  // 2 bytes, signed angle (mm)

    charging_state_ = static_cast<double>(response[16]);  // 1 byte, charging state

    // // Map charging state to a description (handle charging_state as integer for the switch)
    // const char* charging_state_description = "Unknown";
    // switch (static_cast<int>(charging_state)) {
    //     case 0: charging_state_description = "Not Charging"; break;
    //     case 1: charging_state_description = "Charging Recovery"; break;
    //     case 2: charging_state_description = "Charging"; break;
    //     case 3: charging_state_description = "Trickle Charging"; break;
    //     case 4: charging_state_description = "Waiting"; break;
    //     case 5: charging_state_description = "Charging Error"; break;
    // }

    // Unpack 2-byte unsigned values (voltage, current, charge, capacity)
    voltage_ = static_cast<double>((static_cast<uint16_t>(response[17]) << 8) | response[18]);  // 2 bytes, unsigned voltage (mV)
    current_ = static_cast<double>((static_cast<int16_t>(response[19] << 8) | response[20]));  // 2 bytes, signed current (mA)
    temperature_ = static_cast<double>(response[21]);  // 1 byte, temperature (°C)
    charge_ = static_cast<double>((static_cast<uint16_t>(response[22]) << 8) | response[23]);  // 2 bytes, unsigned charge (mAh)
    capacity_ = static_cast<double>((static_cast<uint16_t>(response[24]) << 8) | response[25]);  // 2 bytes, unsigned capacity (mAh)

    // Log parsed data

    RCLCPP_INFO(logger_, "Bumps and Wheeldrops:");
    RCLCPP_INFO(logger_, "  Bump Left: %f", bump_left_);
    RCLCPP_INFO(logger_, "  Bump Right: %f", bump_right_);
    RCLCPP_INFO(logger_, "  Wheel Drop Left: %f", wheeldrops_left_);
    RCLCPP_INFO(logger_, "  Wheel Drop Right: %f", wheeldrops_right_);
    RCLCPP_INFO(logger_, "  Wheel Drop Caster: %f", wheeldrops_caster_);

    RCLCPP_INFO(logger_, "Wall Sensor: %f", wall_);

    RCLCPP_INFO(logger_, "Cliff Sensors:");
    RCLCPP_INFO(logger_, "  Cliff Left: %f", cliff_left_);
    RCLCPP_INFO(logger_, "  Cliff Front Left: %f", cliff_front_left_);
    RCLCPP_INFO(logger_, "  Cliff Front Right: %f", cliff_front_right_);
    RCLCPP_INFO(logger_, "  Cliff Right: %f", cliff_right_);

    RCLCPP_INFO(logger_, "Virtual Wall: %f", virtual_wall_);

    RCLCPP_INFO(logger_, "Motor Overcurrents:");
    RCLCPP_INFO(logger_, "  Side Brush: %f", motor_overcurrents_sidebrush_);
    RCLCPP_INFO(logger_, "  Vacuum: %f", motor_overcurrents_vacuum_);
    RCLCPP_INFO(logger_, "  Main Brush: %f", motor_overcurrents_mainbrush_);
    RCLCPP_INFO(logger_, "  Drive Right: %f", motor_overcurrents_driveright_);
    RCLCPP_INFO(logger_, "  Drive Left: %f", motor_overcurrents_driveleft_);

    RCLCPP_INFO(logger_, "Dirt Detectors:");
    RCLCPP_INFO(logger_, "  Dirt Left: %f", dirt_detector_left_);
    RCLCPP_INFO(logger_, "  Dirt Right: %f", dirt_detector_right_);

    RCLCPP_INFO(logger_, "Remote Control Command: %f", remote_opcode_);  // Assuming it's a numeric opcode
    RCLCPP_INFO(logger_, "Buttons Pressed: %f", buttons_);

    RCLCPP_INFO(logger_, "Distance Traveled: %f meters", distance_);
    RCLCPP_INFO(logger_, "Angle Turned: %f", angle_);

    RCLCPP_INFO(logger_, "Charging State: %f", charging_state_);

    RCLCPP_INFO(logger_, "Battery Information:");
    RCLCPP_INFO(logger_, "  Voltage: %f V", voltage_);
    RCLCPP_INFO(logger_, "  Current: %f A", current_);
    RCLCPP_INFO(logger_, "  Temperature: %f °C", temperature_);
    RCLCPP_INFO(logger_, "  Charge: %f mAh", charge_);
    RCLCPP_INFO(logger_, "  Capacity: %f mAh", capacity_);


    return hardware_interface::return_type::OK;
}



}  // namespace robot_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  robot_hardware_interface::RobotHardwareInterface, hardware_interface::SystemInterface)

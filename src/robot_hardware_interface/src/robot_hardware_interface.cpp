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

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to be ready for read calls and write calls of some interfaces
  RCLCPP_INFO(logger_, "Configuring Hardware interface...");

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

  // Return empty vector for now (fill with your actual joints later)
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
  
  RCLCPP_INFO(logger_, "PACR hardware interface deactivated.");
  
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
  // Checlk if serial port is open
  if (serial_fd_ < 0) {
      RCLCPP_ERROR(logger_, "Serial port not open");
      return hardware_interface::return_type::ERROR;
  }

  // Get the current velocities from the command interfaces 
  int16_t velocity = static_cast<int16_t>(linear_velocity_);  // mm/s
  int16_t radius = static_cast<int16_t>(angular_velocity_);  // Control for turning radius
  int16_t position = static_cast<int16_t>(linear_position_); // mm
  int16_t ang_position = static_cast<int16_t>(angular_position_); // mm

  // Check for velocity limits
  if (velocity > 500) velocity = 500;
  if (velocity < -500) velocity = -500;
  // Check for radius limits
  if (radius > 2000) radius = 2000;
  if (radius < -2000) radius = -2000;

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
      //RCLCPP_INFO(logger_, "Sent DRIVE command: vel=%d mm/s, radius=0x%04X", velocity, radius);
      return hardware_interface::return_type::OK;
  }

  // test communication
    // ---- Read Roomba Sensor Group 3 (10 bytes) ----
  uint8_t sensor_cmd[2] = {142, 2};   // 142 = Sensor, 3 = Group 3 (10 bytes)

  if (::write(serial_fd_, sensor_cmd, 2) != 2) {
      RCLCPP_ERROR(logger_, "Failed to request sensor data (142,2)");
      return hardware_interface::return_type::ERROR;
  }

  usleep(1000); // give Roomba time to respond

  uint8_t sensor_buf[6];
  ssize_t bytes_read = ::read(serial_fd_, sensor_buf, 10);  // Use ssize_t instead of int

  if (bytes_read < 0) { 
      RCLCPP_ERROR(logger_, "read() failed while reading sensor group 3");
      return hardware_interface::return_type::ERROR;
  }
}

}  // namespace robot_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  robot_hardware_interface::RobotHardwareInterface, hardware_interface::SystemInterface)

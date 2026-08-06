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

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

using boost::asio::serial_port;

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

    // Resize vectors to number of joints and set them to 0.0
    hw_states_position_.resize(info_.joints.size(), 0.0);
    hw_states_velocity_.resize(info_.joints.size(), 0.0);
    hw_commands_.resize(info_.joints.size(), 0.0);

    // Resize joint limit vectors
    hw_max_position_.resize(info_.joints.size(), 0.0);
    hw_min_position_.resize(info_.joints.size(), 0.0);

    // Resize sensor state vector
    hw_sensor_states_.resize(info_.sensors[0].state_interfaces.size(), 0.0);

    // Resize joint names vector
    joint_names_.resize(info_.joints.size());

    // GPIO states and commands
    gpio_commands_.resize(info_.gpios.size(), 0.0);
    gpio_states_.resize(info_.gpios.size(), 0.0);

    // Get wheelbase parameter from URDF
    wheelbase_ = std::stod(info_.hardware_parameters["wheelbase"]);
    RCLCPP_INFO(logger_, "wheelbase: %f", wheelbase_);

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

    int idx_gpio = 0;
    for (const auto &gpio : info_.gpios)
    {
      for (const auto &command_interface : gpio.command_interfaces)
      {
        RCLCPP_INFO(logger_, "GPIO %s: command interface %s", gpio.name.c_str(), command_interface.name.c_str());
      }
      for (const auto &state_interface : gpio.state_interfaces)
      {
        RCLCPP_INFO(logger_, "GPIO %s: state interface %s", gpio.name.c_str(), state_interface.name.c_str());
      }
      idx_gpio++;
    }

    // Initialize serial connection
    serial_fd_ = -1;

    // Intialize serial buffer
    serial_buffer_.resize(1024, 0); // Adjust size as needed

    // Initialize state variables
    last_vl_ = 0;
    last_vr_ = 0;
    last_clean_mode_ = 0.0;
    last_led_bits_ = 0.0;
    last_led_color_ = 0.0;
    last_led_intensity_ = 0.0;

    last_motors_cmd_[0] = 144;
    last_motors_cmd_[1] = 0;
    last_motors_cmd_[2] = 0;
    last_motors_cmd_[3] = 0;

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

    static constexpr const char *SERIAL_PORT = "/dev/ttyUSB0"; // on linux it might be /dev/ttyUSB0 or similar, on Windows COM3 or similar
    static constexpr int BAUD_RATE = 115200;

    // Initalize serial
    ser.open(SERIAL_PORT);
    ser.set_option(serial_port::baud_rate(BAUD_RATE));
    ser.set_option(serial_port::character_size(8));
    ser.set_option(serial_port::parity(serial_port::parity::none));
    ser.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    ser.set_option(serial_port::flow_control(serial_port::flow_control::none));

    // Send Roomba startup commands
    boost::asio::write(ser, boost::asio::buffer("\x80", 1));

    // Odometry
    current_pose_x_ = 0.0;
    current_pose_y_ = 0.0;
    current_pose_theta_ = 0.0;

    // Initialize state variables
    last_vl_ = 0;
    last_vr_ = 0;
    last_clean_mode_ = 0.0;

    // Set SCI to FULL mode
    boost::asio::write(ser, boost::asio::buffer("\x84", 1));
    RCLCPP_INFO(logger_, "Roomba placed in FULL mode");

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
    
    // Print joint names
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      RCLCPP_INFO(logger_, "Joint %zu: %s", i, info_.joints[i].name.c_str());
    }

    // Print GPIO names
    for (size_t i = 0; i < info_.gpios.size(); ++i)
    {
      RCLCPP_INFO(logger_, "GPIO %zu: %s", i, info_.gpios[i].name.c_str());
    }

    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
    }

    // Export sensor state interface
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
                info_.gpios[i].name,                     // GPIO name
                info_.gpios[i].state_interfaces[j].name, // Interface type for GPIO
                &gpio_states_[i]));                      // Pointer to the GPIO state variable
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


    // Print joint names
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      RCLCPP_INFO(logger_, "Joint %zu: %s", i, info_.joints[i].name.c_str());
    }

    // Print GPIO names
    for (size_t i = 0; i < info_.gpios.size(); ++i)
    {
      RCLCPP_INFO(logger_, "GPIO %zu: %s", i, info_.gpios[i].name.c_str());
    }

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
                info_.gpios[i].name,                       // GPIO name
                info_.gpios[i].command_interfaces[j].name, // Interface type for GPIO
                &gpio_commands_[i]));                      // Pointer to the GPIO command variable
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
    RCLCPP_INFO(logger_, "Activating...");
    // ============================

    uint8_t stream_cmd[] = {149, 2, 43, 44};

    const int STABILITY_THRESHOLD = 10; // Number of consecutive stable readings required to consider values stable
    const int TIMEOUT_LIMIT = 50;      // Maximum number of iterations before timing out (to avoid infinite loop)

    int stability_counter = 0;
    int timeout_counter = 0;

    // Initialize previous encoder counts to impossible values
    previous_left_encoder_counts_ = 1234;
    previous_right_encoder_counts_ = 1234;

    uint16_t left_wheel_encoder_counts_ = 0;
    uint16_t right_wheel_encoder_counts_ = 0;

    while (timeout_counter < TIMEOUT_LIMIT)
    {
      boost::asio::write(ser, boost::asio::buffer(stream_cmd, 4));

      // Read response from serial port (26 bytes expected)
      uint8_t response_flush[4];
      size_t total_read_2 = 0;

      while (total_read_2 < 4)
      {
        boost::system::error_code ec;
        size_t n = ser.read_some(boost::asio::buffer(response_flush + total_read_2, 4 - total_read_2), ec);
        if (ec)
        {
          RCLCPP_ERROR(logger_, "Error reading from serial port: %s", ec.message().c_str());
        }
        total_read_2 += n;
      }

      // Combine the received bytes into encoder counts
      left_wheel_encoder_counts_ = (static_cast<uint16_t>(response_flush[0]) << 8) |
                                   static_cast<uint16_t>(response_flush[1]);

      right_wheel_encoder_counts_ = (static_cast<uint16_t>(response_flush[2]) << 8) |
                                    static_cast<uint16_t>(response_flush[3]);

      // RCLCPP_INFO(logger_, "Left Encoder: %d, Right Encoder: %d, Stability Counter: %d",left_wheel_encoder_counts_, right_wheel_encoder_counts_, stability_counter);
      //  Check if the encoder values have stabilized
      if (left_wheel_encoder_counts_ == previous_left_encoder_counts_ &&
          right_wheel_encoder_counts_ == previous_right_encoder_counts_)
      {
        stability_counter++; // Increment counter if the values haven't changed
      }
      else
      {
        stability_counter = 0; // Reset the counter if the values have changed
      }

      // If the values have been stable for the required number of iterations, break the loop
      if (stability_counter >= STABILITY_THRESHOLD)
      {
        break;
      }

      // Store the current values for comparison in the next iteration
      previous_left_encoder_counts_ = static_cast<int32_t>(left_wheel_encoder_counts_);
      previous_right_encoder_counts_ = static_cast<int32_t>(right_wheel_encoder_counts_);

      timeout_counter++; // Increment the timeout counter to track iteration limit

      // Small delay to avoid overwhelming the serial communication
      usleep(1000000); // 10 ms
    }

    // Check if the loop timed out
    if (timeout_counter >= TIMEOUT_LIMIT)
    {
      RCLCPP_WARN(logger_, "Timeout reached while waiting for stable encoder values.");
      previous_left_encoder_counts_ = 0;
      previous_right_encoder_counts_ = 0;
      RCLCPP_INFO(logger_, "Encoder values reset to zero.");

    } else{
      // Set encoder counts to zero after stabilization
      previous_left_encoder_counts_ = 0;
      previous_right_encoder_counts_ = 0;

      RCLCPP_INFO(logger_, "Encoder values have stabilized and reset to zero.");
    }

    // ---- Read Roomba Sensor Group 0 (26 bytes) ----

    uint8_t stream_cmd_26[] = {142, 0};
    boost::asio::write(ser, boost::asio::buffer(stream_cmd_26, 2));

    usleep(10000); // wait for 100 ms to allow data to be sent

    // Read response from serial port (26 bytes expected)
    uint8_t response[26];
    size_t total_read = 0;

    while (total_read < 26)
    {
      boost::system::error_code ec;
      size_t n = ser.read_some(boost::asio::buffer(response + total_read, 26 - total_read), ec);
      if (ec)
      {
        RCLCPP_ERROR(logger_, "Error reading from serial port: %s", ec.message().c_str());
      }
      total_read += n;
    }

    // Parse the sensor data (convert to double where needed)
    uint8_t bumps_and_wheeldrops = response[0];
    hw_sensor_states_[0] = static_cast<double>(bumps_and_wheeldrops & 1);        // Bit 0: Bump Right
    hw_sensor_states_[1] = static_cast<double>((bumps_and_wheeldrops >> 1) & 1); // Bit 1: Bump Left
    hw_sensor_states_[2] = static_cast<double>((bumps_and_wheeldrops >> 2) & 1); // Bit 2: Wheel Drop Right
    hw_sensor_states_[3] = static_cast<double>((bumps_and_wheeldrops >> 3) & 1); // Bit 3: Wheel Drop Left
    hw_sensor_states_[4] = static_cast<double>(response[1]);                     // 1 byte, wall sensor
    hw_sensor_states_[5] = static_cast<double>(response[2]);                     // 1 byte, left cliff sensor
    hw_sensor_states_[6] = static_cast<double>(response[3]);                     // 1 byte, front left cliff sensor
    hw_sensor_states_[7] = static_cast<double>(response[4]);                     // 1 byte, front right cliff sensor
    hw_sensor_states_[8] = static_cast<double>(response[5]);                     // 1 byte, right cliff sensor
    hw_sensor_states_[9] = static_cast<double>(response[6]);                     // 1 byte, virtual wall sensor

    uint8_t motor_overcurrents = response[7];
    hw_sensor_states_[10] = static_cast<double>(motor_overcurrents & 1);        // Bit 0: Side Brush
    hw_sensor_states_[11] = 0.0;                                                // Bit 1: Vacuum
    hw_sensor_states_[12] = static_cast<double>((motor_overcurrents >> 2) & 1); // Bit 2: Main Brush
    hw_sensor_states_[13] = static_cast<double>((motor_overcurrents >> 3) & 1); // Bit 3: Drive Right
    hw_sensor_states_[14] = static_cast<double>((motor_overcurrents >> 4) & 1); // Bit 4: Drive Left
    hw_sensor_states_[15] = static_cast<double>(response[8]);                   // 1 byte, dirt detector left
    // double unused1 = static_cast<double>(p[9]);  // 1 byte unused
    hw_sensor_states_[16] = static_cast<double>(response[10]); // 1 byte, remote control command

    uint8_t buttons = response[11];
    hw_sensor_states_[17] = static_cast<double>(buttons & 1);        // Bit 0: Clean
    hw_sensor_states_[18] = static_cast<double>((buttons >> 1) & 1); // Bit 1: Spot
    hw_sensor_states_[19] = static_cast<double>((buttons >> 2) & 1); // Bit 2: Dock
    hw_sensor_states_[20] = static_cast<double>((buttons >> 3) & 1); // Bit 3: Minute
    hw_sensor_states_[21] = static_cast<double>((buttons >> 4) & 1); // Bit 4: Hour
    hw_sensor_states_[22] = static_cast<double>((buttons >> 5) & 1); // Bit 5: Day
    hw_sensor_states_[23] = static_cast<double>((buttons >> 6) & 1); // Bit 6: Schedule
    hw_sensor_states_[24] = static_cast<double>((buttons >> 7) & 1); // Bit 7: Clock

    hw_sensor_states_[25] = static_cast<double>((static_cast<int16_t>(response[12] << 8 | response[13])));  // 2 bytes, signed distance (mm)
    hw_sensor_states_[26] = static_cast<double>((static_cast<int16_t>(response[14] << 8 | response[15])));  // 2 bytes, signed angle (mm)
    hw_sensor_states_[27] = static_cast<double>(response[16]);                                              // 1 byte, charging state
    hw_sensor_states_[28] = static_cast<double>((static_cast<uint16_t>(response[17] << 8) | response[18])); // 2 bytes, unsigned voltage (mV)
    hw_sensor_states_[29] = static_cast<double>((static_cast<int16_t>(response[19] << 8) | response[20]));  // 2 bytes, signed current (mA)
    hw_sensor_states_[30] = static_cast<double>(response[21]);                                              // 1 byte, temperature (°C)
    hw_sensor_states_[31] = static_cast<double>((static_cast<uint16_t>(response[22] << 8) | response[23])); // 2 bytes, unsigned charge (mAh)
    hw_sensor_states_[32] = static_cast<double>((static_cast<uint16_t>(response[24] << 8) | response[25])); // 2 bytes, unsigned capacity (mAh)

    // Odometry
    current_pose_x_ = 0.0;
    current_pose_y_ = 0.0;
    current_pose_theta_ = 0.0;

    // Start the data stream
    // [148] [Number of packets=1] [Packet ID 100]
    uint8_t stream_cmd_2[] = {148, 1, 100};
    boost::asio::write(ser, boost::asio::buffer(stream_cmd_2, 3));

    RCLCPP_INFO(logger_, "Started Sensordata Stream");

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

    boost::system::error_code ec;

    boost::asio::write(ser, boost::asio::buffer(drive_cmd, 5), ec);
    if (ec)
    {
      RCLCPP_ERROR(logger_, "Failed to write drive_cmd during deactivate: %s", ec.message().c_str());
      // Optional: handle recovery or just log
    }

    boost::asio::write(ser, boost::asio::buffer("\x80", 1), ec);
    if (ec)
    {
      RCLCPP_ERROR(logger_, "Failed to write 0x80 during deactivate: %s", ec.message().c_str());
    }

    // Everything done, safe to log
    RCLCPP_INFO(logger_, "Deactivated.");

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RobotHardwareInterface::on_cleanup(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Close serial port
    ser.close();

    RCLCPP_INFO(logger_, "Cleaned up.");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RobotHardwareInterface::on_shutdown(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Close serial port
    ser.close();

    RCLCPP_INFO(logger_, "Shutdown complete.");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Big-endian helper function definitions
  int16_t RobotHardwareInterface::be16(const uint8_t *p)
  {
    return (int16_t)((p[0] << 8) | p[1]);
  }

  uint16_t RobotHardwareInterface::ube16(const uint8_t *p)
  {
    return (uint16_t)((p[0] << 8) | p[1]);
  }

  hardware_interface::return_type RobotHardwareInterface::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // ============================
    // read()
    // read the current state from the robot hardware
    // ============================

    uint16_t previous_left_normalized = previous_left_encoder_counts_ & 0xFFFF;   // normalize to 16 bits
    uint16_t previous_right_normalized = previous_right_encoder_counts_ & 0xFFFF; // normalize to 16 bits

    uint16_t left_wheel_encoder_counts_ = previous_left_normalized;
    uint16_t right_wheel_encoder_counts_ = previous_right_normalized;

    // Temporary read buffer from serial
    uint8_t temp_buf[512];
    boost::system::error_code ec;

    // === Read serial data ===
    size_t n = ser.read_some(boost::asio::buffer(temp_buf, sizeof(temp_buf)), ec);

    if (ec)
    {
      if (ec == boost::asio::error::eof)
      {
        RCLCPP_ERROR(logger_, "Serial port disconnected");
      }
      else
      {
        RCLCPP_ERROR(logger_, "Serial read error: %s", ec.message().c_str());
      }
    }
    else
    {
      // Append new bytes into circular buffer
      for (size_t i = 0; i < n; ++i)
      {
        serial_buffer_.push_back(temp_buf[i]); // automatically overwrites oldest if full
      }
    }

    // === Scan backwards for the newest valid packet ===
    if (serial_buffer_.size() >= 84)
    { // only scan if we have enough bytes
      for (int i = (int)serial_buffer_.size() - 84; i >= 0; --i)
      {
        if (serial_buffer_[i] == 19 && serial_buffer_[i + 1] == 81 && serial_buffer_[i + 2] == 100)
        {
          // Copy packet to contiguous array
          uint8_t pkt[84];
          for (int k = 0; k < 84; ++k)
          {
            pkt[k] = serial_buffer_[i + k];
          }

          // Verify checksum
          uint8_t checksum = 0;
          for (int k = 0; k < 84; ++k)
            checksum += pkt[k];
          if ((checksum & 0xFF) != 0)
            continue;

          uint8_t *p = pkt + 3;

          uint8_t bumps_and_wheeldrops = p[0];
          hw_sensor_states_[0] = static_cast<double>(bumps_and_wheeldrops & 1);        // Bit 0: Bump Right
          hw_sensor_states_[1] = static_cast<double>((bumps_and_wheeldrops >> 1) & 1); // Bit 1: Bump Left
          hw_sensor_states_[2] = static_cast<double>((bumps_and_wheeldrops >> 2) & 1); // Bit 2: Wheel Drop Right
          hw_sensor_states_[3] = static_cast<double>((bumps_and_wheeldrops >> 3) & 1); // Bit 3: Wheel Drop Left

          hw_sensor_states_[4] = static_cast<double>(p[1]); // 1 byte, wall sensor
          hw_sensor_states_[5] = static_cast<double>(p[2]); // 1 byte, left cliff sensor
          hw_sensor_states_[6] = static_cast<double>(p[3]); // 1 byte, front left cliff sensor
          hw_sensor_states_[7] = static_cast<double>(p[4]); // 1 byte, front right cliff sensor
          hw_sensor_states_[8] = static_cast<double>(p[5]); // 1 byte, right cliff sensor
          hw_sensor_states_[9] = static_cast<double>(p[6]); // 1 byte, virtual wall sensor

          uint8_t motor_overcurrents = p[7];
          hw_sensor_states_[10] = static_cast<double>(motor_overcurrents & 1);        // Bit 0: Side Brush
          hw_sensor_states_[11] = 0.0;                                                // Bit 1: Vacuum, not provided in the stream, set to 0.0 for now
          hw_sensor_states_[12] = static_cast<double>((motor_overcurrents >> 2) & 1); // Bit 2: Main Brush
          hw_sensor_states_[13] = static_cast<double>((motor_overcurrents >> 3) & 1); // Bit 3: Drive Right
          hw_sensor_states_[14] = static_cast<double>((motor_overcurrents >> 4) & 1); // Bit 4: Drive Left

          hw_sensor_states_[15] = static_cast<double>(p[8]); // 1 byte, dirt detector left
          //  == 1 Byte unused at p[9] ==
          hw_sensor_states_[16] = static_cast<double>(p[10]); // 1 byte, remote control command

          uint8_t buttons = p[11];
          hw_sensor_states_[17] = static_cast<double>(buttons & 1);        // Bit 0: Clean
          hw_sensor_states_[18] = static_cast<double>((buttons >> 1) & 1); // Bit 1: Spot
          hw_sensor_states_[19] = static_cast<double>((buttons >> 2) & 1); // Bit 2: Dock
          hw_sensor_states_[20] = static_cast<double>((buttons >> 3) & 1); // Bit 3: Minute
          hw_sensor_states_[21] = static_cast<double>((buttons >> 4) & 1); // Bit 4: Hour
          hw_sensor_states_[22] = static_cast<double>((buttons >> 5) & 1); // Bit 5: Day
          hw_sensor_states_[23] = static_cast<double>((buttons >> 6) & 1); // Bit 6: Schedule
          hw_sensor_states_[24] = static_cast<double>((buttons >> 7) & 1); // Bit 7: Clock

          hw_sensor_states_[25] = static_cast<double>((static_cast<int16_t>(p[12] << 8 | p[13])));  // 2 bytes, signed distance (mm)
          hw_sensor_states_[26] = static_cast<double>((static_cast<int16_t>(p[14] << 8 | p[15])));  // 2 bytes, signed angle (mm)
          hw_sensor_states_[27] = static_cast<double>(p[16]);                                       // 1 byte, charging state
          hw_sensor_states_[28] = static_cast<double>((static_cast<uint16_t>(p[17] << 8) | p[18])); // 2 bytes, unsigned voltage (mV)
          hw_sensor_states_[29] = static_cast<double>((static_cast<int16_t>(p[19] << 8) | p[20]));  // 2 bytes, signed current (mA)
          hw_sensor_states_[30] = static_cast<double>(p[21]);                                       // 1 byte, temperature (°C)
          hw_sensor_states_[31] = static_cast<double>(be16(&p[22]));                                // 2 bytes, unsigned charge (mAh)
          hw_sensor_states_[32] = static_cast<double>(be16(&p[24]));                                // 2 bytes, unsigned capacity (mAh)

          hw_sensor_states_[33] = static_cast<double>(be16(&p[26])); // 2 byte, wall signal
          hw_sensor_states_[34] = static_cast<double>(be16(&p[28])); // 2 byte, cliff left signal
          hw_sensor_states_[35] = static_cast<double>(be16(&p[30])); // 2 byte, cliff front left signal
          hw_sensor_states_[36] = static_cast<double>(be16(&p[32])); // 2 byte, cliff front right signal
          hw_sensor_states_[37] = static_cast<double>(be16(&p[34])); // 2 byte, cliff right signal
          //  == 1 Byte unused at p[36] ==
          // == 2 Bytes unused at p[37] and p[38] ==
          hw_sensor_states_[38] = static_cast<double>(p[39]); // 1 byte, charger available
          hw_sensor_states_[39] = static_cast<double>(p[40]); // 1 byte, open interface mode
          hw_sensor_states_[40] = static_cast<double>(p[41]); // 1 byte, song number
          hw_sensor_states_[41] = static_cast<double>(p[42]); // 1 byte, song playing
          hw_sensor_states_[42] = static_cast<double>(p[43]); // 1 byte, oi stream num packets

          hw_sensor_states_[43] = static_cast<double>(be16(&p[44])); // 2 bytes, velocity
          hw_sensor_states_[44] = static_cast<double>(be16(&p[46])); // 2 bytes, radius
          hw_sensor_states_[45] = static_cast<double>(be16(&p[48])); // 2 bytes, velocity right
          hw_sensor_states_[46] = static_cast<double>(be16(&p[50])); // 2 bytes, velocity left

          left_wheel_encoder_counts_ = ube16(&p[52]);  // 2 bytes, encoder counts left
          right_wheel_encoder_counts_ = ube16(&p[54]); // 2 bytes, encoder counts right

          hw_sensor_states_[49] = static_cast<double>(p[56]);        // 1 byte, light bumper
          hw_sensor_states_[50] = static_cast<double>(be16(&p[57])); // 2 byte, light bump left
          hw_sensor_states_[51] = static_cast<double>(be16(&p[59])); // 2 byte, light bump front left
          hw_sensor_states_[52] = static_cast<double>(be16(&p[61])); // 2 byte, light bump center left
          hw_sensor_states_[53] = static_cast<double>(be16(&p[63])); // 2 byte, light bump center right
          hw_sensor_states_[54] = static_cast<double>(be16(&p[65])); // 2 byte, light bump front right
          hw_sensor_states_[55] = static_cast<double>(be16(&p[67])); // 2 byte, light bump right

          hw_sensor_states_[56] = static_cast<double>(p[69]); // 1 byte, ir opcode left
          hw_sensor_states_[57] = static_cast<double>(p[70]); // 1 byte, ir opcode right

          hw_sensor_states_[58] = static_cast<double>(be16(&p[71])); // 2 bytes, left motor current
          hw_sensor_states_[59] = static_cast<double>(be16(&p[73])); // 2 bytes, right motor current
          hw_sensor_states_[60] = static_cast<double>(be16(&p[75])); // 2 bytes, main brush current
          hw_sensor_states_[61] = static_cast<double>(be16(&p[77])); // 2 bytes, side brush current

          hw_sensor_states_[62] = static_cast<double>(p[79]); // 1 byte, stasis

          break; // Exit loop after processing the latest valid packet
        }
      }
    }

    // RCLCPP_INFO(logger_, "Left wheel drop: %f, Right wheel drop: %f", hw_sensor_states_[2], hw_sensor_states_[3]);
    // RCLCPP_INFO(logger_, "Charge and Capacity: %f mAh, %f mAh", hw_sensor_states_[31], hw_sensor_states_[32]);

    constexpr int32_t ENCODER_MAX = 1u << 16;

    // Handle encoder wraparound by calculating the delta and adjusting if it exceeds half the max value
    int32_t left_delta_ = static_cast<int32_t>(left_wheel_encoder_counts_ - previous_left_normalized);
    if (left_delta_ > ENCODER_MAX / 2)
    {
      left_delta_ -= ENCODER_MAX;
    }
    else if (left_delta_ < -static_cast<int32_t>(ENCODER_MAX / 2))
    {
      left_delta_ += ENCODER_MAX;
    }

    // Handle encoder wraparound for right wheel
    int32_t right_delta_ = static_cast<int32_t>(right_wheel_encoder_counts_ - previous_right_normalized);
    if (right_delta_ > ENCODER_MAX / 2)
    {
      right_delta_ -= ENCODER_MAX;
    }
    else if (right_delta_ < -static_cast<int32_t>(ENCODER_MAX / 2))
    {
      right_delta_ += ENCODER_MAX;
    }

    // RCLCPP_INFO(logger_, "Left Wheel: %d, previous: %d, Delta: %d", left_wheel_encoder_counts_ ,previous_left_encoder_counts_, left_delta_);
    // RCLCPP_INFO(logger_, "Right Wheel: %d, previous: %d, Delta: %d", right_wheel_encoder_counts_, previous_right_encoder_counts_, right_delta_);

    previous_left_encoder_counts_ += left_delta_;   // keep full count
    previous_right_encoder_counts_ += right_delta_; // keep full count

    // calculate theta for each wheel
    theta_left_ = (2 * M_PI * previous_left_encoder_counts_) / 508.8;   // in radians
    theta_right_ = (2 * M_PI * previous_right_encoder_counts_) / 508.8; // in radians

    double left_wheel_difference = static_cast<double>(left_delta_);
    double right_wheel_difference = static_cast<double>(right_delta_);

    double tick_to_distance_ = (72 * 3.14159) / 508.8; // in mm
    double left_distance = left_wheel_difference * tick_to_distance_;
    double right_distance = right_wheel_difference * tick_to_distance_; // in mm

    distance_ = (left_distance + right_distance) / 2.0;
    angle_ = (right_distance - left_distance) / 2.0;

    // Update odometry
    current_pose_theta_ += (2 * angle_) / (wheelbase_ * 1000);
    current_pose_x_ += distance_ * cos(current_pose_theta_) / 1000; // convert mm to meters
    current_pose_y_ += distance_ * sin(current_pose_theta_) / 1000; // convert mm to meters

    // RCLCPP_INFO(logger_, "Current Pose x: %f, y: %f, theta: %f", current_pose_x_, current_pose_y_, current_pose_theta_);

    // write pose state interfaces
    hw_states_position_[0] = theta_left_;  // left_wheel_joint position
    hw_states_position_[1] = theta_right_; // right_wheel_joint position

    // Provide the full encoder counts as sensor states
    hw_sensor_states_[47] = previous_left_encoder_counts_;  // 2 bytes, signed left encoder counts
    hw_sensor_states_[48] = previous_right_encoder_counts_; // 2 bytes, signed right encoder counts

    // Provide manually calculated odometry as sensor states for testing
    hw_sensor_states_[63] = current_pose_x_;     // x position
    hw_sensor_states_[64] = current_pose_y_;     // y position
    hw_sensor_states_[65] = current_pose_theta_; // theta position

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type RobotHardwareInterface::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // ============================
    // write()
    // write the current command to the robot hardware
    // ============================

    // ======================================
    // Send GPIOs
    // ======================================
    uint8_t gpio_cmd_array[] = {
        144,
        static_cast<uint8_t>(static_cast<int>(gpio_commands_[0])), // Cast from float to int, then to uint8_t
        static_cast<uint8_t>(static_cast<int>(gpio_commands_[1])),
        static_cast<uint8_t>(static_cast<int>(gpio_commands_[2]))};

    if (memcmp(gpio_cmd_array, last_motors_cmd_, 4) != 0)
    { 
      // RCLCPP_INFO(logger_, "Sending GPIO command: %d %d %d", gpio_cmd_array[1], gpio_cmd_array[2], gpio_cmd_array[3]);
      boost::asio::write(ser, boost::asio::buffer(gpio_cmd_array, 4));
      memcpy(last_motors_cmd_, gpio_cmd_array, 4);
    }

    // ======================================
    // Send CLEAN MODE command
    // ======================================

    if (gpio_commands_[3] != last_clean_mode_)
    {
      const double raw_cmd = gpio_commands_[3];
      
      if (raw_cmd < 0 || raw_cmd > 255)
      {
        RCLCPP_WARN(logger_, "Invalid clean mode: %d", static_cast<int>(raw_cmd));
      }
      else
      {
        uint8_t clean_cmd = static_cast<uint8_t>(raw_cmd);

        try
        {
          boost::asio::write(ser, boost::asio::buffer(&clean_cmd, 1));
          RCLCPP_INFO(logger_, "Sent CLEAN MODE command: %u", clean_cmd);
          last_clean_mode_ = raw_cmd;
        }
        catch (const boost::system::system_error &e)
        {
          RCLCPP_ERROR(logger_, "Serial write threw an exception: %s", e.what());
        }
      }
    }

    // ======================================
    // Send LEDS command (opcode 139)
    // ======================================
    // Leds command: [139] [Led Bits (0-63)] [Color (0-255)] [Intensity (0-255)]
    // Led Bits: bit 0=Clean, 1=Spot, 2=Dock, 3=Check Robot, 4=Scheduling, 5=Dirt Detect
    // Check if LED commands have changed (gpio_commands_[4], [5], [6])
    
    if (gpio_commands_[4] != last_led_bits_ || 
        gpio_commands_[5] != last_led_color_ || 
        gpio_commands_[6] != last_led_intensity_)
    {
      const double led_bits_raw = gpio_commands_[4];
      const double led_color_raw = gpio_commands_[5];
      const double led_intensity_raw = gpio_commands_[6];
      
      // Validate ranges
      if (led_bits_raw < 0 || led_bits_raw > 63 ||
          led_color_raw < 0 || led_color_raw > 255 ||
          led_intensity_raw < 0 || led_intensity_raw > 255)
      {
        RCLCPP_WARN(logger_, 
                    "Invalid LED values: bits=%f (0-63), color=%f (0-255), intensity=%f (0-255)",
                    led_bits_raw, led_color_raw, led_intensity_raw);
      }
      else
      {
        uint8_t led_cmd_array[] = {
            139,
            static_cast<uint8_t>(static_cast<int>(led_bits_raw)),
            static_cast<uint8_t>(static_cast<int>(led_color_raw)),
            static_cast<uint8_t>(static_cast<int>(led_intensity_raw))};
        
        try
        {
          boost::asio::write(ser, boost::asio::buffer(led_cmd_array, 4));
          RCLCPP_INFO(logger_, "Sent LEDS command: bits=%u, color=%u, intensity=%u",
                      led_cmd_array[1], led_cmd_array[2], led_cmd_array[3]);
          
          // Update last sent values
          last_led_bits_ = led_bits_raw;
          last_led_color_ = led_color_raw;
          last_led_intensity_ = led_intensity_raw;
        }
        catch (const boost::system::system_error &e)
        {
          RCLCPP_ERROR(logger_, "Serial write threw an exception: %s", e.what());
        }
      }
    }

    // ======================================
    // Send DRIVE command
    // ======================================

    const double vl = hw_commands_[0];
    const double vr = hw_commands_[1];

    int16_t left_wheel_velocity_ = static_cast<int16_t>(
        std::clamp(vl * 1000.0 * 0.036, -400.0, 400.0));

    int16_t right_wheel_velocity_ = static_cast<int16_t>(
        std::clamp(vr * 1000.0 * 0.036, -400.0, 400.0));

    // For Info print
    double velocity_ = (left_wheel_velocity_ + right_wheel_velocity_) / 2.0; // calculate velocity
    double radius_ = 0.0;
    if (left_wheel_velocity_ != right_wheel_velocity_)
    {
      radius_ = ((left_wheel_velocity_ + right_wheel_velocity_) / 2.0) * wheelbase_ / (left_wheel_velocity_ - right_wheel_velocity_);
    }

    uint8_t drive_cmd[5];

    if (vl != last_vl_ || vr != last_vr_)
    {
      drive_cmd[0] = 145;                                                       // Command identifier for Drive Direct
      drive_cmd[1] = static_cast<uint8_t>((right_wheel_velocity_ >> 8) & 0xFF); // High byte of velocity
      drive_cmd[2] = static_cast<uint8_t>(right_wheel_velocity_ & 0xFF);        // Low byte of velocity
      drive_cmd[3] = static_cast<uint8_t>((left_wheel_velocity_ >> 8) & 0xFF);  // High byte of radius
      drive_cmd[4] = static_cast<uint8_t>(left_wheel_velocity_ & 0xFF);         // Low byte of radius

      try
      {
        boost::asio::write(ser, boost::asio::buffer(drive_cmd, 5));
        RCLCPP_INFO(logger_, "Sent D cmd: vel=%.2f mm/s, r=%.2f mm, right=%.2f mm/s, left=%.2f mm/s",
                    velocity_, radius_, vr, vl);
      }
      catch (const boost::system::system_error &e)
      {
        RCLCPP_ERROR(logger_, "Serial write threw an exception: %s", e.what());
      }

      // Update last sent values
      last_vl_ = vl;
      last_vr_ = vr;
    }

    return hardware_interface::return_type::OK;
  }

} // namespace robot_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    robot_hardware_interface::RobotHardwareInterface, hardware_interface::SystemInterface)

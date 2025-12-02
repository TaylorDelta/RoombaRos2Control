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

#ifndef ROBOT_HARDWARE_INTERFACE__ROBOT_HARDWARE_INTERFACE_HPP_
#define ROBOT_HARDWARE_INTERFACE__ROBOT_HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <rclcpp/rclcpp.hpp>

namespace robot_hardware_interface
{
class RobotHardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;

  rclcpp::Logger logger_{rclcpp::get_logger("HardwareInterface")};

  int serial_fd_;
  double linear_position_;
  double linear_velocity_;
  double linear_acceleration_;
  double angular_position_;
  double angular_velocity_;
  double angular_acceleration_;




  int16_t last_velocity_;
  int16_t last_radius_;

  // uint8_t byte_bumps_wheeldrops_;  // 1 byte
  // uint8_t byte_wall_;              // 1 byte
  // uint8_t byte_cliff_left_;        // 1 byte
  // uint8_t byte_cliff_front_left_;  // 1 byte
  // uint8_t byte_cliff_front_right_; // 1 byte
  // uint8_t byte_cliff_right_;       // 1 byte
  // uint8_t byte_virtual_wall_;      // 1 byte
  // uint8_t byte_motor_overcurrents_; // 1 byte
  // uint8_t byte_dirt_left_;         // 1 byte
  // uint8_t byte_dirt_right_;        // 1 byte
  // uint8_t byte_remote_opcode_;     // 1 byte
  // uint8_t byte_buttons_;           // 1 byte
  // int16_t byte_distance_;          // 2 bytes
  // int16_t byte_angle_;             // 2 bytes
  // uint8_t byte_charging_state_;    // 1 byte
  // uint16_t byte_voltage_;          // 2 bytes
  // int16_t byte_current_;           // 2 bytes
  // int8_t byte_temperature_;        // 1 byte
  // uint16_t byte_charge_;           // 2 bytes
  // uint16_t byte_capacity_;         // 2 bytes


  
  // State Variables for Sensor Data
  double bump_right_;               // State of right bumper (on/off)
  double bump_left_;                // State of left bumper (on/off)

  double wheeldrops_right_;         // State of right wheel drop sensor (on/off)
  double wheeldrops_left_;          // State of left wheel drop sensor (on/off)
  double wheeldrops_caster_;        // State of caster wheel drop sensor (on/off)

  double wall_;                     // State of the wall sensor (on/off)
  double cliff_left_;               // State of left cliff sensor (on/off)
  double cliff_front_left_;         // State of front left cliff sensor (on/off)
  double cliff_front_right_;        // State of front right cliff sensor (on/off)
  double cliff_right_;              // State of right cliff sensor (on/off)
  double virtual_wall_;             // State of virtual wall sensor (on/off)

  double motor_overcurrents_sidebrush_; // Overcurrent status for sidebrush motor (on/off)
  double motor_overcurrents_vacuum_;    // Overcurrent status for vacuum motor (on/off)
  double motor_overcurrents_mainbrush_; // Overcurrent status for mainbrush motor (on/off)
  double motor_overcurrents_driveright_; // Overcurrent status for right drive motor (on/off)
  double motor_overcurrents_driveleft_;  // Overcurrent status for left drive motor (on/off)

  double dirt_detector_left_;         // Dirt detector state for left side (on/off)
  double dirt_detector_right_;        // Dirt detector state for right side (on/off)

  double remote_opcode_;              // Remote opcode (could be an integer representing command)
  double buttons_;                   // State of buttons (pressed/unpressed)

  double distance_;                // Distance sensor reading (in meters, for example)
  double angle_;                   // Angle sensor reading (in radians or degrees)
  
  double charging_state_;             // Charging state (could be an integer enum or state)
  double voltage_;                 // Voltage sensor reading (in volts)
  double current_;                 // Current sensor reading (in amperes)
  double temperature_;             // Temperature sensor reading (in degrees Celsius)

  double charge_;                  // Current battery charge (in mAh, for example)
  double capacity_;                // Battery capacity (in mAh)
};

}  // namespace robot_hardware_interface

#endif  // ROBOT_HARDWARE_INTERFACE__ROBOT_HARDWARE_INTERFACE_HPP_

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

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/circular_buffer.hpp>


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

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
private:

  // Big-endian helpers for converting data
  static int16_t be16(const uint8_t* p); // Function declaration (signature)
  static uint16_t ube16(const uint8_t* p); // Function declaration (signature)
  
  // Hardware state and command vectors
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  
  // Joint limit vectors
  std::vector<double> hw_max_position_;
  std::vector<double> hw_min_position_;

  // Sensor state vector
  std::vector<double> hw_sensor_states_;

  // Joint names
  std::vector<std::string> joint_names_;

  // Motors states and commands
  std::vector<double> gpio_commands_;
  std::vector<double> gpio_states_;

  // Wheelbase parameter
  double wheelbase_;


  rclcpp::Logger logger_{rclcpp::get_logger("HardwareInterface")};
  
  // File descriptor for serial communication
  int serial_fd_; 
  boost::asio::io_service io_service_;  // Boost IO service for serial communication
  boost::asio::serial_port ser{io_service_};  // Serial port object

  // Boost circular buffer for serial communication
  boost::circular_buffer<uint8_t> serial_buffer_{2048};

  //Odometry
  double current_pose_x_;
  double current_pose_y_;
  double current_pose_theta_;

  double theta_left_;
  double theta_right_;

  // Last sent velocity and radius to avoid redundant commands
  double last_vl_;
  double last_vr_;
  double last_clean_mode_;
  double last_led_bits_;       // Store last sent LED bits (0-63)
  double last_led_color_;      // Store last sent LED color (0-255)
  double last_led_intensity_;  // Store last sent LED intensity (0-255)

  uint8_t last_motors_cmd_[4]; // Store last sent motors command to avoid redundant writes
  
  double distance_;                // Distance sensor reading (in meters, for example)
  double angle_;                   // Angle sensor reading (in radians or degrees)

  uint32_t previous_left_encoder_counts_; // Store previous left encoder counts
  uint32_t previous_right_encoder_counts_; // Store previous right encoder counts
};

}  // namespace robot_hardware_interface

#endif  // ROBOT_HARDWARE_INTERFACE__ROBOT_HARDWARE_INTERFACE_HPP_

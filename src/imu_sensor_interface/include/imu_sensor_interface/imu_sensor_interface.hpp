// Copyright (c) 2026, Taylor Delta
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.
#ifndef IMU_SENSOR_INTERFACE__IMU_SENSOR_INTERFACE_HPP_
#define IMU_SENSOR_INTERFACE__IMU_SENSOR_INTERFACE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace imu_sensor_interface
{
class IMUSensorInterface : public hardware_interface::SensorInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
private:
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
};

}  // namespace imu_sensor_interface

#endif  // IMU_SENSOR_INTERFACE__IMU_SENSOR_INTERFACE_HPP_

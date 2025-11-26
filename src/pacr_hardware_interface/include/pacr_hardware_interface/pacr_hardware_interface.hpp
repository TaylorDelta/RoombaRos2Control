#ifndef PACR_HARDWARE_INTERFACE_HPP_
#define PACR_HARDWARE_INTERFACE_HPP_

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace pacr_hardware_interface
{
class PACRHardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
private:
  int serial_fd_;
  rclcpp::Logger logger_{rclcpp::get_logger("PACRHardwareInterface")};
};
}  // namespace pacr_hardware_interface

#endif  // PACR_HARDWARE_INTERFACE_HPP_

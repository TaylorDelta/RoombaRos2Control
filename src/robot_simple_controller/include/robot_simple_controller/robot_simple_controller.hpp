// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

//
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
//

#ifndef ROBOT_SIMPLE_CONTROLLER__ROBOT_SIMPLE_CONTROLLER_HPP_
#define ROBOT_SIMPLE_CONTROLLER__ROBOT_SIMPLE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "robot_simple_controller_parameters.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_srvs/srv/set_bool.hpp"

// TODO(anyone): Replace with controller specific messages
#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
#include "nav_msgs/msg/odometry.hpp"




namespace robot_simple_controller
{
// name constants for state interfaces
static constexpr size_t STATE_MY_ITFS = 0;

// name constants for command interfaces
static constexpr size_t CMD_MY_ITFS = 0;

// TODO(anyone: example setup for control mode (usually you will use some enums defined in messages)
enum class control_mode_type : std::uint8_t
{
  FAST = 0,
  SLOW = 1,
};

class RobotSimpleController : public controller_interface::ControllerInterface
{
public:
  RobotSimpleController();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // TODO(anyone): replace the state and command message types
  using ControllerReferenceMsg = control_msgs::msg::JointJog;
  using ControllerModeSrvType = std_srvs::srv::SetBool;
  using ControllerStateMsg = control_msgs::msg::JointControllerState;
  using GeometryMsgTwist = geometry_msgs::msg::Twist;
  using StdMsgInt32 = std_msgs::msg::Int32;


protected:
  std::shared_ptr<robot_simple_controller::ParamListener> param_listener_;
  robot_simple_controller::Params params_;

  std::vector<std::string> state_joints_;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

  // Subscribe to /cmd_vel topic
  rclcpp::Subscription<GeometryMsgTwist>::SharedPtr cmd_vel_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<GeometryMsgTwist>> input_cmd_vel_;

  // Subscribe to /robot/clean_mode topic
  rclcpp::Subscription<StdMsgInt32>::SharedPtr clean_mode_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<StdMsgInt32>> input_clean_mode_;

  // Publish to /odom topic
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<nav_msgs::msg::Odometry>> output_odom_;


  rclcpp::Service<ControllerModeSrvType>::SharedPtr set_slow_control_mode_service_;
  realtime_tools::RealtimeBuffer<control_mode_type> control_mode_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;

private:
  // callback for topic interface
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
  void cmd_vel_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg);
  void clean_mode_callback(const std::shared_ptr<std_msgs::msg::Int32> msg);
};

}  // namespace robot_simple_controller

#endif  // ROBOT_SIMPLE_CONTROLLER__ROBOT_SIMPLE_CONTROLLER_HPP_

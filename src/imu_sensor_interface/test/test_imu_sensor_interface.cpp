// Copyright (c) 2026, Taylor Delta
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include <gmock/gmock.h>

#include <string>

#include "hardware_interface/resource_manager.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

class TestIMUSensorInterface : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // TODO(anyone): Extend this description to your robot
    imu_sensor_interface_2dof_ =
      R"(
        <ros2_control name="IMUSensorInterface2dof" type="sensor">
          <hardware>
            <plugin>imu_sensor_interface/IMUSensorInterface</plugin>
          </hardware>
          <joint name="joint1">
            <state_interface name="position"/>
            <param name="initial_position">1.57</param>
          </joint>
          <joint name="joint2">
            <state_interface name="position"/>
            <param name="initial_position">0.7854</param>
          </joint>
        </ros2_control>
    )";
  }

  std::string imu_sensor_interface_2dof_;
};

TEST_F(TestIMUSensorInterface, load_imu_sensor_interface_2dof)
{
  auto urdf = ros2_control_test_assets::urdf_head + imu_sensor_interface_2dof_ +
              ros2_control_test_assets::urdf_tail;
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf));
}

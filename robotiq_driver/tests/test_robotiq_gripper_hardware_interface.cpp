// Copyright (c) 2023 PickNik, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include <hardware_interface/resource_manager.hpp>

#include <rclcpp/node.hpp>

namespace robotiq_driver::test
{

/**
 * This test generates a minimal xacro robot configuration and loads the
 * hardware interface plugin.
 */
TEST(TestRobotiqGripperHardwareInterface, load_urdf)
{
  std::string urdf =
      R"(
        <?xml version="1.0" encoding="utf-8"?>
        <robot name="test_robot">
          <link name="robotiq_85_base_link"/>
          <link name="robotiq_85_left_knuckle_link"/>
          <joint name="robotiq_85_left_knuckle_joint" type="revolute">
            <parent link="robotiq_85_base_link" />
            <child link="robotiq_85_left_knuckle_link" />
            <axis xyz="0 -1 0" />
            <origin xyz="0.03060114 0.0 0.05490452" rpy="0 0 0" />
            <limit lower="0.0" upper="0.8" velocity="0.5" effort="50" />
          </joint>
          <ros2_control name="robotiq_driver_ros2_control" type="system">
            <hardware>
              <plugin>robotiq_driver/RobotiqGripperHardwareInterface</plugin>
              <param name="gripper_speed_multiplier">1.0</param>
              <param name="gripper_force_multiplier">0.5</param>
              <param name="COM_port">/dev/ttyUSB0</param>
              <param name="gripper_closed_position">0.7929</param>
            </hardware>
            <joint name="robotiq_85_left_knuckle_joint">
              <command_interface name="position" />
              <state_interface name="position">
                <param name="initial_value">0.7929</param>
              </state_interface>
              <state_interface name="velocity"/>
            </joint>
          </ros2_control>
        </robot>
        )";

  rclcpp::Node node{ "test_robotiq_gripper_hardware_interface" };

  // Initialize the resource manager
  hardware_interface::ResourceManager rm(urdf, node.get_node_clock_interface(), node.get_node_logging_interface());

  // Check interfaces
  EXPECT_EQ(1u, rm.system_components_size());
}

}  // namespace robotiq_driver::test

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

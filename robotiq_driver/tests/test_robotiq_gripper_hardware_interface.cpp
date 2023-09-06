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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>

#include <robotiq_driver/default_driver_factory.hpp>
#include <robotiq_driver/hardware_interface.hpp>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <ros2_control_test_assets/components_urdfs.hpp>
#include <ros2_control_test_assets/descriptions.hpp>

namespace robotiq_driver::test
{

/**
 * This test generates a minimal xacro robot configuration and loads the
 * hardware interface plugin.
 */
TEST(TestRobotiqGripperHardwareInterface, load_urdf)
{
  std::string urdf_control_ =
      R"(
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
       )";

  auto urdf = ros2_control_test_assets::urdf_head + urdf_control_ + ros2_control_test_assets::urdf_tail;
  hardware_interface::ResourceManager rm(urdf);

  // Check interfaces
  EXPECT_EQ(1u, rm.system_components_size());
}

}  // namespace robotiq_driver::test

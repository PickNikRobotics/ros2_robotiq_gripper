// Copyright 2022 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <boost/lockfree/spsc_queue.hpp>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "robotiq_driver/visibility_control.h"
#include "robotiq_driver/robotiq_gripper_interface.hpp"

namespace robotiq_driver
{
class RobotiqGripperHardwareInterface : public hardware_interface::ActuatorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RobotiqGripperHardwareInterface)

  ROBOTIQ_DRIVER_PUBLIC
  RobotiqGripperHardwareInterface();

  ROBOTIQ_DRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  ROBOTIQ_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROBOTIQ_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROBOTIQ_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  ROBOTIQ_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  ROBOTIQ_DRIVER_PUBLIC
  hardware_interface::return_type read() override;

  ROBOTIQ_DRIVER_PUBLIC
  hardware_interface::return_type write() override;

private:
  double gripper_position_;
  double gripper_velocity_;
  double gripper_position_command_;
  std::unique_ptr<RobotiqGripperInterface> gripper_interface_;

  double gripper_closed_pos_;
  std::string com_port_;

  enum class CommandType
  {
    READ,
    WRITE
  };
  std::thread command_interface_;
  bool command_interface_is_running_;
  boost::lockfree::spsc_queue<uint8_t> write_commands_;
  boost::lockfree::spsc_queue<uint8_t> responses_;
};

}  // namespace robotiq_driver

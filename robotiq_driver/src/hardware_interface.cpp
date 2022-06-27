// Copyright 2022 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "robotiq_driver/hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

const auto kLogger = rclcpp::get_logger("RobotiqGripperHardwareInterface");
const double kGripperOpenPos = 0.7929;

namespace robotiq_driver
{
CallbackReturn RobotiqGripperHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  gripper_position_ = std::numeric_limits<double>::quiet_NaN();
  gripper_velocity_ = std::numeric_limits<double>::quiet_NaN();
  gripper_position_command_ = std::numeric_limits<double>::quiet_NaN();

  const hardware_interface::ComponentInfo& joint = info_.joints[0];

  // There is one command interface: position.
  if (joint.command_interfaces.size() != 1)
  {
    RCLCPP_FATAL(kLogger, "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                 joint.command_interfaces.size());
    return CallbackReturn::ERROR;
  }

  if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_FATAL(kLogger, "Joint '%s' has %s command interfaces found. '%s' expected.", joint.name.c_str(),
                 joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
    return CallbackReturn::ERROR;
  }

  // There are two state interfaces: position and velocity.
  if (joint.state_interfaces.size() != 2)
  {
    RCLCPP_FATAL(kLogger, "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                 joint.state_interfaces.size());
    return CallbackReturn::ERROR;
  }

  for (int i = 0; i < 2; ++i)
  {
    if (!(joint.state_interfaces[i].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[i].name == hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_FATAL(kLogger, "Joint '%s' has %s state interface. Expected %s or %s.", joint.name.c_str(),
                   joint.state_interfaces[i].name.c_str(), hardware_interface::HW_IF_POSITION,
                   hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  // Create the interface to the gripper.
  gripper_interface_ = std::make_unique<RobotiqGripperInterface>("/dev/ttyUSB0");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotiqGripperHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &gripper_position_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &gripper_velocity_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotiqGripperHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[0].name, hardware_interface::HW_IF_POSITION, &gripper_position_command_));

  return command_interfaces;
}

CallbackReturn RobotiqGripperHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // set some default values for joints
  if (std::isnan(gripper_position_))
  {
    gripper_position_ = 0;
    gripper_velocity_ = 0;
    gripper_position_command_ = 0;
  }

  // Activate the gripper.
  gripper_interface_->deactivateGripper();
  if (!gripper_interface_->activateGripper())
  {
    RCLCPP_FATAL(kLogger, "Failed to activate gripper.");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(kLogger, "Successfully activated!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotiqGripperHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotiqGripperHardwareInterface::read()
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(kLogger, "Reading...");

  // getGripperPosition() returns 0x00 when the gripper is fully open, and 0xFF when it is fully closed.
  gripper_position_ = (1 - gripper_interface_->getGripperPosition() / double(0xFF)) * kGripperOpenPos;

  RCLCPP_INFO(kLogger, "Got position %.5f for joint '%s'!", gripper_position_, info_.joints[0].name.c_str());
  RCLCPP_INFO(kLogger, "Got velocity %.5f for joint '%s'!", gripper_velocity_, info_.joints[0].name.c_str());

  RCLCPP_INFO(kLogger, "Joints successfully read!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotiqGripperHardwareInterface::write()
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(kLogger, "Writing...please wait...");

  // Simulate sending commands to the hardware
  RCLCPP_INFO(kLogger, "Got command %.5f for joint '%s'!", gripper_position_command_, info_.joints[0].name.c_str());

  // For the gripper interface, a position command of 0xFF fully closes the gripper, and 0x00 fully opens it.
  uint8_t gripper_pos = (1 - gripper_position_command_ / kGripperOpenPos) * 0xFF;
  gripper_interface_->setGripperPosition(gripper_pos);

  RCLCPP_INFO(kLogger, "Joints successfully written!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace robotiq_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(robotiq_driver::RobotiqGripperHardwareInterface, hardware_interface::ActuatorInterface)

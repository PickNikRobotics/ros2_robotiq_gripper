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

constexpr uint8_t kGripperMinPos = 3;
constexpr uint8_t kGripperMaxPos = 230;
constexpr uint8_t kGripperRange = kGripperMaxPos - kGripperMinPos;

const auto kLogger = rclcpp::get_logger("RobotiqGripperHardwareInterface");

namespace robotiq_driver
{
RobotiqGripperHardwareInterface::RobotiqGripperHardwareInterface() : write_commands_(20), responses_(20)
{
}

CallbackReturn RobotiqGripperHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Read parameters.
  gripper_closed_pos_ = stod(info_.hardware_parameters["gripper_closed_position"]);
  com_port_ = info_.hardware_parameters["COM_port"];
  double gripper_speed = stod(info_.hardware_parameters["gripper_speed_multiplier"]);
  double gripper_force = stod(info_.hardware_parameters["gripper_force_multiplier"]);

  // Speed and force must lie between 0.0 and 1.0.
  gripper_speed = std::min(1.0, std::max(0.0, gripper_speed));
  gripper_force = std::min(1.0, std::max(0.0, gripper_force));

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
  gripper_interface_ = std::make_unique<RobotiqGripperInterface>(com_port_);
  gripper_interface_->setSpeed(gripper_speed * 0xFF);
  gripper_interface_->setForce(gripper_force * 0xFF);

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

  command_interface_is_running_ = true;

  command_interface_ = std::thread([this] {
    // Read from and write to the gripper at 100 Hz.
    auto io_interval = std::chrono::milliseconds(10);
    auto last_io = std::chrono::high_resolution_clock::now();

    while (command_interface_is_running_)
    {
      auto now = std::chrono::high_resolution_clock::now();
      if (now - last_io > io_interval) {
        // Write the latest command to the gripper.
        std::optional<double> cmd = std::nullopt;
        while (write_commands_.read_available())
        {
          cmd = write_commands_.front();
          write_commands_.pop();
        }
        if (cmd.has_value()) {
          this->gripper_interface_->setGripperPosition(cmd.value());
        }

        // Read the state of the gripper.
        this->responses_.push(this->gripper_interface_->getGripperPosition());

        last_io = now;
      }
    }
  });

  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotiqGripperHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  command_interface_is_running_ = false;
  command_interface_.join();

  // Deactivate the gripper.
  gripper_interface_->deactivateGripper();

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotiqGripperHardwareInterface::read()
{
  while (responses_.read_available())
  {
    gripper_position_ = gripper_closed_pos_ * (responses_.front() - kGripperMinPos) / kGripperRange;
    responses_.pop();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotiqGripperHardwareInterface::write()
{
  double gripper_pos = (gripper_position_command_ / gripper_closed_pos_) * kGripperRange + kGripperMinPos;
  gripper_pos = std::max(std::min(gripper_pos, 255.0), 0.0);
  write_commands_.push(uint8_t(gripper_pos));

  return hardware_interface::return_type::OK;
}

}  // namespace robotiq_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(robotiq_driver::RobotiqGripperHardwareInterface, hardware_interface::ActuatorInterface)

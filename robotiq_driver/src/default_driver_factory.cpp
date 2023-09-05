// Copyright (c) 2022 PickNik, Inc.
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

#include <cmath>
#include <rclcpp/logging.hpp>
#include <robotiq_driver/default_driver.hpp>
#include <robotiq_driver/default_driver_factory.hpp>

namespace robotiq_driver
{

const auto kLogger = rclcpp::get_logger("DefaultDriverFactory");

std::unique_ptr<Driver> DefaultDriverFactory::create(
  const hardware_interface::HardwareInfo & info) const
{
  double gripper_speed = stod(info.hardware_parameters.at("gripper_speed_multiplier"));
  double gripper_force = stod(info.hardware_parameters.at("gripper_force_multiplier"));

  // Speed and force must lie between 0.0 and 1.0.
  gripper_speed = std::min(1.0, std::max(0.0, gripper_speed));
  gripper_force = std::min(1.0, std::max(0.0, gripper_force));

  auto driver = create_driver(info);
  driver->set_speed(gripper_speed * 0xFF);
  driver->set_force(gripper_force * 0xFF);

  return driver;
}

std::unique_ptr<Driver> DefaultDriverFactory::create_driver(
  const hardware_interface::HardwareInfo & info) const
{
  std::string com_port = info.hardware_parameters.at("COM_port");
  return std::make_unique<DefaultDriver>(com_port);
}
}  // namespace robotiq_driver

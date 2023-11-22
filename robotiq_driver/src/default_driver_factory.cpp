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

#include <robotiq_driver/default_driver_factory.hpp>
#include <robotiq_driver/default_driver.hpp>
#include <robotiq_driver/default_serial.hpp>
#include <robotiq_driver/default_serial_factory.hpp>
#include <robotiq_driver/fake/fake_driver.hpp>

#include <rclcpp/logging.hpp>

namespace robotiq_driver
{
const auto kLogger = rclcpp::get_logger("DefaultDriverFactory");

constexpr auto kSlaveAddressParamName = "slave_address";
constexpr uint8_t kSlaveAddressParamDefault = 0x09;

constexpr auto kGripperSpeedMultiplierParamName = "gripper_speed_multiplier";
constexpr double kGripperSpeedMultiplierParamDefault = 1.0;

constexpr auto kGripperForceMultiplierParamName = "gripper_force_multiplier";
constexpr double kGripperForceMultiplierParamDefault = 1.0;

constexpr auto kUseDummyParamName = "use_dummy";
constexpr auto kUseDummyParamDefault = "false";

std::unique_ptr<Driver> DefaultDriverFactory::create(const hardware_interface::HardwareInfo& info) const
{
  RCLCPP_INFO(kLogger, "Reading %s...", kSlaveAddressParamName);
  // Convert base-16 address stored as a string (for example, "0x9") into an integer
  const uint8_t slave_address =
      info.hardware_parameters.count(kSlaveAddressParamName) ?
          static_cast<uint8_t>(std::stoul(info.hardware_parameters.at(kSlaveAddressParamName), nullptr, 16)) :
          kSlaveAddressParamDefault;
  RCLCPP_INFO(kLogger, "%s: %d", kSlaveAddressParamName, slave_address);

  RCLCPP_INFO(kLogger, "Reading %s...", kGripperSpeedMultiplierParamName);
  double gripper_speed = info.hardware_parameters.count(kGripperSpeedMultiplierParamName) ?
                             std::clamp(stod(info.hardware_parameters.at(kGripperSpeedMultiplierParamName)), 0.0, 1.0) :
                             kGripperSpeedMultiplierParamDefault;
  RCLCPP_INFO(kLogger, "%s: %fs", kGripperSpeedMultiplierParamName, gripper_speed);

  RCLCPP_INFO(kLogger, "Reading %s...", kGripperForceMultiplierParamName);
  double gripper_force = info.hardware_parameters.count(kGripperForceMultiplierParamName) ?
                             std::clamp(stod(info.hardware_parameters.at(kGripperForceMultiplierParamName)), 0.0, 1.0) :
                             kGripperForceMultiplierParamDefault;
  RCLCPP_INFO(kLogger, "%s: %fs", kGripperForceMultiplierParamName, gripper_force);

  auto driver = create_driver(info);
  driver->set_slave_address(slave_address);
  driver->set_speed(gripper_speed * 0xFF);
  driver->set_force(gripper_force * 0xFF);

  return driver;
}

std::unique_ptr<Driver> DefaultDriverFactory::create_driver(const hardware_interface::HardwareInfo& info) const
{
  // We give the user an option to startup a dummy gripper for testing purposes.
  if (info.hardware_parameters.count(kUseDummyParamName) &&
      info.hardware_parameters.at(kUseDummyParamName) != kUseDummyParamDefault)
  {
    RCLCPP_INFO(kLogger, "You are connected to a dummy driver, not a real hardware.");
    return std::make_unique<FakeDriver>();
  }
  else
  {
    auto serial = DefaultSerialFactory().create(info);
    return std::make_unique<DefaultDriver>(std::move(serial));
  }
}
}  // namespace robotiq_driver

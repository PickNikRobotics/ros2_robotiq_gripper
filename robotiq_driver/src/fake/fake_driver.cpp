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

#include <robotiq_driver/fake/fake_driver.hpp>

#include <rclcpp/logging.hpp>

namespace robotiq_driver
{
const auto kLogger = rclcpp::get_logger("FakeDriver");

void FakeDriver::set_slave_address(uint8_t slave_address)
{
  slave_address_ = slave_address;
  RCLCPP_INFO(kLogger, "slave_address set to: %d", slave_address);
}

bool FakeDriver::connect()
{
  connected_ = true;
  RCLCPP_INFO(kLogger, "Gripper connected.");
  return true;
}

void FakeDriver::disconnect()
{
  RCLCPP_INFO(kLogger, "Gripper disconnected.");
  connected_ = false;
}

void FakeDriver::activate()
{
  RCLCPP_INFO(kLogger, "Gripper activated.");
  activated_ = true;
}

void FakeDriver::deactivate()
{
  RCLCPP_INFO(kLogger, "Gripper deactivated.");
  activated_ = false;
}

void FakeDriver::set_gripper_position(uint8_t position)
{
  position_ = position;
}

uint8_t FakeDriver::get_gripper_position()
{
  return position_;
}

bool FakeDriver::gripper_is_moving()
{
  return gripper_is_moving_;
}

void FakeDriver::set_speed(uint8_t speed)
{
  RCLCPP_INFO(kLogger, "Set gripper speed.");
  speed_ = speed;
}

void FakeDriver::set_force(uint8_t force)
{
  RCLCPP_INFO(kLogger, "Set gripper force.");
  force_ = force;
}

}  // namespace robotiq_driver

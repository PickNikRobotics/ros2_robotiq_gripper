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

#pragma once

#include <robotiq_driver/driver.hpp>

namespace robotiq_driver
{
/**
 * This is a fake driver that can be used for testing interactions with the
 * hardware interface or the controller without being connected to the real
 * hardware. At the moment the fake driver is very basic but it can be
 * improved to behave as close as possible to the real hardware.
 * To use this driver you have to enable the following parameter in your
 * hardware interface configuration in the robot URDF.
 *
 * <!-- Set use_dummy to true to connect to a dummy driver. -->
 * <param name="use_dummy">true</param>
 */
class FakeDriver : public Driver
{
public:
  void set_slave_address(uint8_t slave_address) override;
  bool connect() override;
  void disconnect() override;
  void activate() override;
  void deactivate() override;
  void set_gripper_position(uint8_t position) override;
  uint8_t get_gripper_position() override;
  bool gripper_is_moving() override;
  void set_speed(uint8_t speed) override;
  void set_force(uint8_t force) override;

private:
  uint8_t slave_address_ = 0x00;
  bool connected_ = false;
  bool activated_ = false;
  uint8_t position_ = 0;
  bool gripper_is_moving_ = false;
  uint8_t speed_ = 0;
  uint8_t force_ = 0;
};

}  // namespace robotiq_driver

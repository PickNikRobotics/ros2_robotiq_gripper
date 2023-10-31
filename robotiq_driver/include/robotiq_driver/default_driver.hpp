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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <robotiq_driver/driver.hpp>
#include <robotiq_driver/serial.hpp>

/**
 * @brief This class is responsible for communicating with the gripper via a serial port, and maintaining a record of
 * the gripper's current state.
 *
 */
namespace robotiq_driver
{
class DefaultDriver : public Driver
{
public:
  explicit DefaultDriver(std::unique_ptr<Serial> serial);

  bool connect() override;
  void disconnect() override;

  void set_slave_address(uint8_t slave_address) override;

  /** Activate the gripper with the specified operation mode and parameters. */
  void activate() override;

  /** Deactivate the gripper. */
  void deactivate() override;

  /**
   * @brief Commands the gripper to move to the desired position.
   * @param pos A value between 0x00 (fully open) and 0xFF (fully closed).
   */
  void set_gripper_position(uint8_t pos) override;

  /**
   * @brief Return the current position of the gripper.
   * @throw serial::IOException on failure to successfully communicate with gripper port
   * @return uint8_t A value between 0x00 (fully open) and 0xFF (fully closed).
   */
  uint8_t get_gripper_position() override;

  /**
   * @brief Returns true if the gripper is currently moving, false otherwise.
   *
   */
  bool gripper_is_moving() override;

  /**
   * @brief Set the speed of the gripper.
   * @param speed A value between 0x00 (stopped) and 0xFF (full speed).
   */
  void set_speed(uint8_t speed) override;

  /**
   * @brief Set how forcefully the gripper opens or closes.
   * @param force A value between 0x00 (no force) or 0xFF (maximum force).
   */
  void set_force(uint8_t force) override;

private:
  /**
   * With this command we send a request and wait for a response of given size.
   * Behind the scene, if the response is not received, the software makes an attempt
   * to resend the command up to 5 times before returning an empty response.
   * @param request The command request.
   * @param response_size The response expected size.
   * @return The response or an empty vector if an en error occurred.
   */
  std::vector<uint8_t> send(const std::vector<uint8_t>& request, size_t response_size) const;

  std::vector<uint8_t> create_read_command(uint16_t first_register, uint8_t num_registers);
  std::vector<uint8_t> create_write_command(uint16_t first_register, const std::vector<uint16_t>& data);

  /**
   * @brief Read the current status of the gripper, and update member variables as appropriate.
   *
   * @throw serial::IOException on failure to successfully communicate with gripper port
   */
  void update_status();

  std::unique_ptr<Serial> serial_ = nullptr;
  uint8_t slave_address_;

  ActivationStatus activation_status_;
  ActionStatus action_status_;
  GripperStatus gripper_status_;
  ObjectDetectionStatus object_detection_status_;

  uint8_t gripper_position_;
  uint8_t commanded_gripper_speed_;
  uint8_t commanded_gripper_force_;
};
}  // namespace robotiq_driver

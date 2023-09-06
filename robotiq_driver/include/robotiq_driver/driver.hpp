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

#include <cstdint>

/**
 * @brief This interface describes how to communicate with the gripper hardware.
 */
namespace robotiq_driver
{
class Driver
{
public:
  enum class ActivationStatus
  {
    RESET,
    ACTIVE
  };

  enum class ActionStatus
  {
    STOPPED,
    MOVING
  };

  enum class GripperStatus
  {
    RESET,
    IN_PROGRESS,
    COMPLETED,
  };

  enum class ObjectDetectionStatus
  {
    MOVING,
    OBJECT_DETECTED_OPENING,
    OBJECT_DETECTED_CLOSING,
    AT_REQUESTED_POSITION
  };

  virtual void set_slave_address(uint8_t slave_address) = 0;

  /** Connect to the gripper serial connection. */
  virtual bool connect() = 0;

  /** Disconnect from the gripper serial connection. */
  virtual void disconnect() = 0;

  /**
   * @brief Activates the gripper.
   * @throw serial::IOException on failure to successfully communicate with gripper port
   */
  virtual void activate() = 0;

  /**
   * @brief Deactivates the gripper.
   * @throw serial::IOException on failure to successfully communicate with gripper port
   */
  virtual void deactivate() = 0;

  /**
   * @brief Commands the gripper to move to the desired position.
   * @param pos A value between 0x00 (fully open) and 0xFF (fully closed).
   */
  virtual void set_gripper_position(uint8_t pos) = 0;

  /**
   * @brief Return the current position of the gripper.
   *
   * @throw serial::IOException on failure to successfully communicate with gripper port
   *
   * @return uint8_t A value between 0x00 (fully open) and 0xFF (fully closed).
   */
  virtual uint8_t get_gripper_position() = 0;

  /**
   * @brief Returns true if the gripper is currently moving, false otherwise.
   */
  virtual bool gripper_is_moving() = 0;

  /**
   * @brief Set the speed of the gripper.
   *
   * @param speed A value between 0x00 (stopped) and 0xFF (full speed).
   */
  virtual void set_speed(uint8_t speed) = 0;

  /**
   * @brief Set how forcefully the gripper opens or closes.
   * @param force A value between 0x00 (no force) or 0xFF (maximum force).
   */
  virtual void set_force(uint8_t force) = 0;
};
}  // namespace robotiq_driver

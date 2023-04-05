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

#include <string>
#include <vector>

#include <serial/serial.h>

/**
 * @brief This class is responsible for communicating with the gripper via a serial port, and maintaining a record of
 * the gripper's current state.
 *
 */
class RobotiqGripperInterface
{
public:
  RobotiqGripperInterface(const std::string& com_port = "/dev/ttyUSB0", uint8_t slave_id = 0x09);

  /**
   * @brief Activates the gripper.
   *
   * @throw std::runtime_error on failure to successfully communicate with gripper port
   */
  void activateGripper();

  /**
   * @brief Deactivates the gripper.
   *
   * @throw std::runtime_error on failure to successfully communicate with gripper port
   */
  void deactivateGripper();

  /**
   * @brief Commands the gripper to move to the desired position.
   *
   * @param pos A value between 0x00 (fully open) and 0xFF (fully closed).
   */
  void setGripperPosition(uint8_t pos);

  /**
   * @brief Return the current position of the gripper.
   *
   * @throw std::runtime_error on failure to successfully communicate with gripper port
   * 
   * @return uint8_t A value between 0x00 (fully open) and 0xFF (fully closed).
   */
  uint8_t getGripperPosition();

  /**
   * @brief Returns true if the gripper is currently moving, false otherwise.
   *
   */
  bool gripperIsMoving();

  /**
   * @brief Set the speed of the gripper.
   *
   * @param speed A value between 0x00 (stopped) and 0xFF (full speed).
   */
  void setSpeed(uint8_t speed);

  /**
   * @brief Set how forcefully the gripper opens or closes.
   *
   * @param force A value between 0x00 (no force) or 0xFF (maximum force).
   */
  void setForce(uint8_t force);

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

private:

  std::vector<uint8_t> createReadCommand(uint16_t first_register, uint8_t num_registers);
  std::vector<uint8_t> createWriteCommand(uint16_t first_register, const std::vector<uint16_t>& data);
  
  /**
   * @brief read response from the gripper.
   *
   * @param num_bytes Number of bytes to be read from device port.
   * @throw std::runtime_error on failure to successfully communicate with gripper port
   */
  std::vector<uint8_t> readResponse(size_t num_bytes);

  /**
   * @brief Send a command to the gripper.
   *
   * @param cmd The command.
   * @throw std::runtime_error on failure to successfully communicate with gripper port
   */
  void sendCommand(const std::vector<uint8_t>& cmd);

  /**
   * @brief Read the current status of the gripper, and update member variables as appropriate.
   *
   * @throw std::runtime_error on failure to successfully communicate with gripper port
   */
  void updateStatus();

  serial::Serial port_;
  uint8_t slave_id_;
  std::vector<uint8_t> read_command_;

  ActivationStatus activation_status_;
  ActionStatus action_status_;
  GripperStatus gripper_status_;
  ObjectDetectionStatus object_detection_status_;

  uint8_t gripper_position_;
  uint8_t commanded_gripper_speed_;
  uint8_t commanded_gripper_force_;
};

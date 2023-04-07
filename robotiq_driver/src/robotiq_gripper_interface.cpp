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

#include "robotiq_driver/robotiq_gripper_interface.hpp"
#include "robotiq_driver/crc.hpp"

#include <iostream>
#include <stdexcept>
#include <thread>
#include <chrono>

constexpr int kBaudRate = 115200;
constexpr auto kTimeoutMilliseconds = 1000;

constexpr uint8_t kReadFunctionCode = 0x03;
constexpr uint16_t kFirstOutputRegister = 0x07D0;
constexpr uint16_t kNumOutputRegisters = 0x0006;
// The response to a read request consists of:
//   slave ID (1 byte)
//   function code (1 byte)
//   number of data bytes (1 byte)
//   data bytes (2 bytes per register)
//   CRC (2 bytes)
constexpr int kReadResponseSize = 2 * kNumOutputRegisters + 5;

constexpr uint8_t kWriteFunctionCode = 0x10;
constexpr uint16_t kActionRequestRegister = 0x03E8;
// The response to a write command consists of:
//   slave ID (1 byte)
//   function code (1 byte)
//   address of the first register that was written (2 bytes)
//   number of registers written (2 bytes)
//   CRC (2 bytes)
constexpr int kWriteResponseSize = 8;

constexpr size_t kResponseHeaderSize = 3;
constexpr size_t kGripperStatusIndex = 0;
constexpr size_t kPositionIndex = 4;

static uint8_t getFirstByte(uint16_t val)
{
  return (val & 0xFF00) >> 8;
}

static uint8_t getSecondByte(uint16_t val)
{
  return val & 0x00FF;
}

RobotiqGripperInterface::RobotiqGripperInterface(const std::string& com_port, uint8_t slave_id)
  : port_(com_port, kBaudRate, serial::Timeout::simpleTimeout(kTimeoutMilliseconds))
  , slave_id_(slave_id)
  , read_command_(createReadCommand(kFirstOutputRegister, kNumOutputRegisters))
  , commanded_gripper_speed_(0x80)
  , commanded_gripper_force_(0x80)
{
  if (!port_.isOpen())
  {
    const auto error_msg = "Failed to open gripper port.";
    THROW(serial::IOException, error_msg.c_str());
  }
}

void RobotiqGripperInterface::activateGripper()
{
  const auto cmd = createWriteCommand(kActionRequestRegister, { 0x0100, 0x0000, 0x0000 }  // set rACT to 1, clear all
                                                                                          // other registers.
  );

  try
  {
    sendCommand(cmd);
    readResponse(kWriteResponseSize);

    updateStatus();

    if (gripper_status_ == GripperStatus::COMPLETED)
    {
      return;
    }

    while (gripper_status_ == GripperStatus::IN_PROGRESS)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      updateStatus();
    }
  }
  catch (const serial::IOException& e)
  {
    // catch connection error and rethrow
    std::cerr << "Failed to activate gripper";
    throw;
  }
}

void RobotiqGripperInterface::deactivateGripper()
{
  const auto cmd = createWriteCommand(kActionRequestRegister, { 0x0000, 0x0000, 0x0000 });
  try
  {
    sendCommand(cmd);
    readResponse(kWriteResponseSize);
  }
  catch (const serial::IOException& e)
  {
    // catch connection error and rethrow
    std::cerr << "Failed to activate gripper";
    throw;
  }
}

void RobotiqGripperInterface::setGripperPosition(uint8_t pos)
{
  uint8_t action_register = 0x09;
  uint8_t gripper_options_1 = 0x00;
  uint8_t gripper_options_2 = 0x00;

  const auto cmd =
      createWriteCommand(kActionRequestRegister,
                         { uint16_t(action_register << 8 | gripper_options_1), uint16_t(gripper_options_2 << 8 | pos),
                           uint16_t(commanded_gripper_speed_ << 8 | commanded_gripper_force_) });
  try
  {
    sendCommand(cmd);
    readResponse(kWriteResponseSize);
  }
  catch (const serial::IOException& e)
  {
    // catch connection error and rethrow
    std::cerr << "Failed to set gripper position\n";
    throw;
  }
}

uint8_t RobotiqGripperInterface::getGripperPosition()
{
  try
  {
    updateStatus();
  }
  catch (const serial::IOException& e)
  {
    // catch connection error and rethrow
    std::cerr << "Failed to get gripper position\n";
    throw;
  }

  return gripper_position_;
}

bool RobotiqGripperInterface::gripperIsMoving()
{
  try
  {
    updateStatus();
  }
  catch (const serial::IOException& e)
  {
    // catch connection error and rethrow
    std::cerr << "Failed to get gripper position\n";
    throw;
  }

  return object_detection_status_ == ObjectDetectionStatus::MOVING;
}

std::vector<uint8_t> RobotiqGripperInterface::createReadCommand(uint16_t first_register, uint8_t num_registers)
{
  std::vector<uint8_t> cmd = { slave_id_,
                               kReadFunctionCode,
                               getFirstByte(first_register),
                               getSecondByte(first_register),
                               getFirstByte(num_registers),
                               getSecondByte(num_registers) };
  auto crc = computeCRC(cmd);
  cmd.push_back(getFirstByte(crc));
  cmd.push_back(getSecondByte(crc));
  return cmd;
}

void RobotiqGripperInterface::setSpeed(uint8_t speed)
{
  commanded_gripper_speed_ = speed;
}

void RobotiqGripperInterface::setForce(uint8_t force)
{
  commanded_gripper_force_ = force;
}

std::vector<uint8_t> RobotiqGripperInterface::createWriteCommand(uint16_t first_register,
                                                                 const std::vector<uint16_t>& data)
{
  uint16_t num_registers = data.size();
  uint8_t num_bytes = 2 * num_registers;

  std::vector<uint8_t> cmd = { slave_id_,
                               kWriteFunctionCode,
                               getFirstByte(first_register),
                               getSecondByte(first_register),
                               getFirstByte(num_registers),
                               getSecondByte(num_registers),
                               num_bytes };
  for (auto d : data)
  {
    cmd.push_back(getFirstByte(d));
    cmd.push_back(getSecondByte(d));
  }

  auto crc = computeCRC(cmd);
  cmd.push_back(getFirstByte(crc));
  cmd.push_back(getSecondByte(crc));

  return cmd;
}

std::vector<uint8_t> RobotiqGripperInterface::readResponse(size_t num_bytes_requested)
{
  std::vector<uint8_t> response;
  size_t num_bytes_read = port_.read(response, num_bytes_requested);

  if (num_bytes_read != num_bytes_requested)
  {
    const auto error_msg =
        "Requested " + std::to_string(num_bytes_requested) + " bytes, but only got " + std::to_string(num_bytes_read);
    THROW(serial::IOException, error_msg.c_str());
  }

  return response;
}

void RobotiqGripperInterface::sendCommand(const std::vector<uint8_t>& cmd)
{
  size_t num_bytes_written = port_.write(cmd);
  port_.flush();
  if (num_bytes_written != cmd.size())
  {
    const auto error_msg = "Attempted to write " + std::to_string(cmd.size()) + " bytes, but only wrote " +
                           std::to_string(num_bytes_written);
    THROW(serial::IOException, error_msg.c_str());
  }
}

void RobotiqGripperInterface::updateStatus()
{
  // Tell the gripper that we want to read its status.
  try
  {
    sendCommand(read_command_);

    const auto response = readResponse(kReadResponseSize);

    // Process the response.
    uint8_t gripper_status_byte = response[kResponseHeaderSize + kGripperStatusIndex];

    // Activation status.
    activation_status_ = ((gripper_status_byte & 0x01) == 0x00) ? ActivationStatus::RESET : ActivationStatus::ACTIVE;

    // Action status.
    action_status_ = ((gripper_status_byte & 0x08) == 0x00) ? ActionStatus::STOPPED : ActionStatus::MOVING;

    // Gripper status.
    switch ((gripper_status_byte & 0x30) >> 4)
    {
      case 0x00:
        gripper_status_ = GripperStatus::RESET;
        break;
      case 0x01:
        gripper_status_ = GripperStatus::IN_PROGRESS;
        break;
      case 0x03:
        gripper_status_ = GripperStatus::COMPLETED;
        break;
    }

    // Object detection status.
    switch ((gripper_status_byte & 0xC0) >> 6)
    {
      case 0x00:
        object_detection_status_ = ObjectDetectionStatus::MOVING;
        break;
      case 0x01:
        object_detection_status_ = ObjectDetectionStatus::OBJECT_DETECTED_OPENING;
        break;
      case 0x02:
        object_detection_status_ = ObjectDetectionStatus::OBJECT_DETECTED_CLOSING;
        break;
      case 0x03:
        object_detection_status_ = ObjectDetectionStatus::AT_REQUESTED_POSITION;
        break;
    }

    // Read the current gripper position.
    gripper_position_ = response[kResponseHeaderSize + kPositionIndex];
  }
  catch (const serial::IOException& e)
  {
    std::cerr << "Failed to update gripper status.\n";
    throw;
  }
}

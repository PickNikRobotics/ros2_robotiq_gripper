#include "robotiq_driver/robotiq_gripper_interface.hpp"
#include "robotiq_driver/crc.hpp"

#include <iostream>
#include <thread>
#include <chrono>

constexpr int kBaudRate = 115200;
constexpr auto kTimeoutMilliseconds = 1000;

constexpr uint8_t kReadFunctionCode = 0x03;
constexpr uint16_t kFirstOutputRegister = 0x07D0;
constexpr uint16_t kNumOutputRegisters = 0x0006;

constexpr uint8_t kWriteFunctionCode = 0x10;
constexpr uint16_t kActionRequestRegister = 0x03E8;

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
    std::cerr << "Failed to open gripper port.\n";
    return;
  }
}

bool RobotiqGripperInterface::activateGripper()
{
  auto cmd = createWriteCommand(kActionRequestRegister, { 0x0100, 0x0000, 0x0000 }  // set rACT to 1, clear all
                                                                                    // other registers.
  );

  size_t num_bytes_written = port_.write(cmd);
  if (num_bytes_written != cmd.size())
  {
    std::cerr << "Attempted to write " << cmd.size() << " bytes, but only wrote " << num_bytes_written << ".\n";
    return false;
  }

  readResponse(8);

  updateStatus();
  while (gripper_status_ == GripperStatus::IN_PROGRESS)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    updateStatus();
  }

  return gripper_status_ == GripperStatus::COMPLETED;
}

void RobotiqGripperInterface::deactivateGripper()
{
  auto cmd = createWriteCommand(kActionRequestRegister, { 0x0000, 0x0000, 0x0000 });

  size_t num_bytes_written = port_.write(cmd);
  if (num_bytes_written != cmd.size())
  {
    std::cerr << "Attempted to write " << cmd.size() << " bytes, but only wrote " << num_bytes_written << ".\n";
    return;
  }

  readResponse(8);
}

void RobotiqGripperInterface::setGripperPosition(uint8_t pos)
{
  uint8_t action_register = 0x09;
  uint8_t gripper_options_1 = 0x00;
  uint8_t gripper_options_2 = 0x00;

  auto cmd = createWriteCommand(kActionRequestRegister,
                                { action_register << 8 | gripper_options_1, gripper_options_2 << 8 | pos,
                                  commanded_gripper_speed_ << 8 | commanded_gripper_force_ });

  size_t num_bytes_written = port_.write(cmd);
  if (num_bytes_written != cmd.size())
  {
    std::cerr << "Attempted to write " << cmd.size() << " bytes, but only wrote " << num_bytes_written << ".\n";
    return;
  }

  readResponse(8);
}

uint8_t RobotiqGripperInterface::getGripperPosition()
{
  updateStatus();
  return gripper_position_;
}

bool RobotiqGripperInterface::gripperIsMoving()
{
  updateStatus();
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
  auto crc = computeCRC(cmd.begin(), cmd.end());
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

  auto crc = computeCRC(cmd.begin(), cmd.end());
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
    std::cerr << "Requested " << num_bytes_requested << " bytes, but only got " << num_bytes_read << ".\n";
  }

  return response;
}

void RobotiqGripperInterface::updateStatus()
{
  // Tell the gripper that we want to read its status.
  size_t num_bytes_written = port_.write(read_command_);
  if (num_bytes_written != read_command_.size())
  {
    std::cerr << "Attempted to write " << read_command_.size() << " bytes, but only wrote " << num_bytes_written
              << ".\n";
    return;
  }

  auto response = readResponse(17);

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

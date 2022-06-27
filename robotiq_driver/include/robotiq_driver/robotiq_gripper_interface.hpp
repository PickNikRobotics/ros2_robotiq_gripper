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
   * @return true The gripper was activated successfully.
   * @return false The gripper was not activated.
   */
  bool activateGripper();

  /**
   * @brief Deactivates the gripper.
   *
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
   * @return uint8_t A value between 0x00 (fully open) and 0xFF (fully closed).
   */
  uint8_t getGripperPosition();

  /**
   * @brief Returns true if the gripper is currently moving, false otherwise.
   *
   */
  bool gripperIsMoving();

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
    COMPLETED
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
  std::vector<uint8_t> readResponse(size_t num_bytes);

  /**
   * @brief Read the current status of the gripper, and update member variables as appropriate.
   *
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
};

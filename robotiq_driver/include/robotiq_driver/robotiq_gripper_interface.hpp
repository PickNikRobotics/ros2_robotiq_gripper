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

private:
  std::vector<uint8_t> createReadCommand(uint16_t first_register, uint8_t num_registers);
  std::vector<uint8_t> createWriteCommand(uint16_t first_register, const std::vector<uint16_t>& data);
  std::vector<uint8_t> readResponse(size_t num_bytes);

  serial::Serial port_;
  uint8_t slave_id_;
  std::vector<uint8_t> read_command_;
};

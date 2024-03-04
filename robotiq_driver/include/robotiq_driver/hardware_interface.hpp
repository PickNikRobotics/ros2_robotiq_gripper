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

#include <atomic>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <robotiq_driver/visibility_control.hpp>

#include <robotiq_driver/driver.hpp>
#include <robotiq_driver/driver_factory.hpp>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace robotiq_driver
{
class RobotiqGripperHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RobotiqGripperHardwareInterface)

  /**
   * Default constructor.
   */
  ROBOTIQ_DRIVER_PUBLIC
  RobotiqGripperHardwareInterface();

  ROBOTIQ_DRIVER_PUBLIC
  ~RobotiqGripperHardwareInterface();

  /**
   * Constructor with a driver factory. This method is used for testing.
   * @param driver_factory The driver that interact with the hardware.
   */
  explicit RobotiqGripperHardwareInterface(std::unique_ptr<DriverFactory> driver_factory);

  /**
   * Initialization of the hardware interface from data parsed from the
   * robot's URDF.
   * @param hardware_info Structure with data from URDF.
   * @returns CallbackReturn::SUCCESS if required data are provided and can be
   * parsed or CallbackReturn::ERROR if any error happens or data are missing.
   */
  ROBOTIQ_DRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  /**
   * Connect to the hardware.
   * @param previous_state The previous state.
   * @returns CallbackReturn::SUCCESS if required data are provided and can be
   * parsed or CallbackReturn::ERROR if any error happens or data are missing.
   */
  ROBOTIQ_DRIVER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * This method exposes position and velocity of joints for reading.
   */
  ROBOTIQ_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * This method exposes the joints targets for writing.
   */
  ROBOTIQ_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * This method is invoked when the hardware is connected.
   * @param previous_state Unconfigured, Inactive, Active or Finalized.
   * @returns CallbackReturn::SUCCESS or CallbackReturn::ERROR.
   */
  ROBOTIQ_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * This method is invoked when the hardware is disconnected.
   * @param previous_state Unconfigured, Inactive, Active or Finalized.
   * @returns CallbackReturn::SUCCESS or CallbackReturn::ERROR.
   */
  ROBOTIQ_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * Read data from the hardware.
   */
  ROBOTIQ_DRIVER_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  /**
   * Write data to hardware.
   */
  ROBOTIQ_DRIVER_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

protected:
  // Interface to send binary data to the hardware using the serial port.
  std::unique_ptr<Driver> driver_;

  // Factory to create the driver during the initialization step.
  std::unique_ptr<DriverFactory> driver_factory_;

  // We use a thread to read/write to the driver so that we dont block the hardware_interface read/write.
  std::thread communication_thread_;
  std::atomic<bool> communication_thread_is_running_;
  void background_task();

  double gripper_closed_pos_ = 0.0;

  static constexpr double NO_NEW_CMD_ = std::numeric_limits<double>::quiet_NaN();

  double gripper_position_ = 0.0;
  double gripper_velocity_ = 0.0;
  double gripper_position_command_ = 0.0;

  std::atomic<uint8_t> write_command_;
  std::atomic<uint8_t> write_force_;
  std::atomic<uint8_t> write_speed_;
  std::atomic<uint8_t> gripper_current_state_;

  double reactivate_gripper_cmd_ = 0.0;
  std::atomic<bool> reactivate_gripper_async_cmd_;
  double reactivate_gripper_response_ = 0.0;
  double gripper_force_ = 0.0;
  double gripper_speed_ = 0.0;
  std::atomic<std::optional<bool>> reactivate_gripper_async_response_;
};

}  // namespace robotiq_driver

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

#include <iostream>
#include <robotiq_driver/robotiq_gripper_interface.hpp>
#include <thread>

constexpr auto kComPort = "/dev/ttyUSB0";
constexpr auto kSlaveID = 0x09;

int main()
{
  try {
    RobotiqGripperInterface gripper(kComPort, kSlaveID);

    std::cout << "Deactivating gripper...\n";
    gripper.deactivateGripper();

    std::cout << "Activating gripper...\n";
    gripper.activateGripper();

    std::cout << "Gripper successfully activated.\n";

    std::cout << "Closing gripper...\n";
    gripper.setGripperPosition(0xFF);
    while (gripper.gripperIsMoving()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "Opening gripper...\n";
    gripper.setGripperPosition(0x00);
    while (gripper.gripperIsMoving()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "Half closing gripper...\n";
    gripper.setGripperPosition(0x80);
    while (gripper.gripperIsMoving()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "Opening gripper...\n";
    gripper.setGripperPosition(0x00);
    while (gripper.gripperIsMoving()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "Decreasing gripper speed...\n";
    gripper.setSpeed(0x0F);

    std::cout << "Closing gripper...\n";
    gripper.setGripperPosition(0xFF);
    while (gripper.gripperIsMoving()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "Increasing gripper speed...\n";
    gripper.setSpeed(0xFF);

    std::cout << "Opening gripper...\n";
    gripper.setGripperPosition(0x00);
    while (gripper.gripperIsMoving()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  } catch (const serial::IOException & e) {
    std::cout << "Failed to communicating with the Gripper. Please check the Gripper connection";
    return 1;
  }

  return 0;
}

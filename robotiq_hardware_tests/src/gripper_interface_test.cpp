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

#include <chrono>
#include <thread>
#include <iostream>
#include <memory>

#include <robotiq_driver/default_driver.hpp>
#include <robotiq_driver/default_serial.hpp>

constexpr auto kComPort = "/dev/ttyUSB0";
constexpr auto kBaudRate = 115200;
constexpr auto kTimeout = 1;
constexpr auto kSlaveAddress = 0x09;

using robotiq_driver::DefaultDriver;
using robotiq_driver::DefaultSerial;

int main()
{
  try
  {
    auto serial = std::make_unique<DefaultSerial>();
    serial->set_port(kComPort);
    serial->set_baudrate(kBaudRate);
    serial->set_timeout(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(kTimeout)));

    auto driver = std::make_unique<DefaultDriver>(std::move(serial));
    driver->set_slave_address(kSlaveAddress);

    std::cout << "Deactivating gripper...\n";
    driver->deactivate();

    std::cout << "Activating gripper...\n";
    driver->activate();

    std::cout << "Gripper successfully activated.\n";

    std::cout << "Closing gripper...\n";
    driver->set_gripper_position(0xFF);
    while (driver->gripper_is_moving())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "Opening gripper...\n";
    driver->set_gripper_position(0x00);
    while (driver->gripper_is_moving())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "Half closing gripper...\n";
    driver->set_gripper_position(0x80);
    while (driver->gripper_is_moving())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "Opening gripper...\n";
    driver->set_gripper_position(0x00);
    while (driver->gripper_is_moving())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "Decreasing gripper speed...\n";
    driver->set_speed(0x0F);

    std::cout << "Closing gripper...\n";
    driver->set_gripper_position(0xFF);
    while (driver->gripper_is_moving())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "Increasing gripper speed...\n";
    driver->set_speed(0xFF);

    std::cout << "Opening gripper...\n";
    driver->set_gripper_position(0x00);
    while (driver->gripper_is_moving())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }
  catch (std::exception e)
  {
    std::cout << "Failed to communicating with the gripper: " << e.what();
    return 1;
  }

  return 0;
}

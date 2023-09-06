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

#include "command_line_utility.hpp"

constexpr auto kComPort = "/dev/ttyUSB0";
constexpr auto kBaudRate = 115200;
constexpr auto kTimeout = 1;
constexpr auto kSlaveAddress = 0x09;

using robotiq_driver::DefaultDriver;
using robotiq_driver::DefaultSerial;

int main(int argc, char* argv[])
{
  CommandLineUtility cli;

  std::string port = kComPort;
  cli.registerHandler(
      "--port", [&port](const char* value) { port = value; }, false);

  int baudrate = kBaudRate;
  cli.registerHandler(
      "--baudrate", [&baudrate](const char* value) { baudrate = std::stoi(value); }, false);

  double timeout = kTimeout;
  cli.registerHandler(
      "--timeout", [&timeout](const char* value) { timeout = std::stod(value); }, false);

  int slave_address = kSlaveAddress;
  cli.registerHandler(
      "--slave-address", [&slave_address](const char* value) { slave_address = std::stoi(value); }, false);

  cli.registerHandler("-h", [&]() {
    std::cout << "Usage: ./set_relative_pressure [OPTIONS]\n"
              << "Options:\n"
              << "  --port VALUE                      Set the com port (default " << kComPort << ")\n"
              << "  --baudrate VALUE                  Set the baudrate (default " << kBaudRate << "bps)\n"
              << "  --timeout VALUE                   Set the read/write timeout (default " << kTimeout << "ms)\n"
              << "  --slave-address VALUE             Set the slave address (default " << kSlaveAddress << ")\n"
              << "  -h                                Show this help message\n";
    exit(0);
  });

  if (!cli.parse(argc, argv))
  {
    return 1;
  }

  try
  {
    auto serial = std::make_unique<DefaultSerial>();
    serial->set_port(port);
    serial->set_baudrate(baudrate);
    serial->set_timeout(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(timeout)));

    auto driver = std::make_unique<DefaultDriver>(std::move(serial));
    driver->set_slave_address(slave_address);

    std::cout << "Using the following parameters: " << std::endl;
    std::cout << " - port: " << port << std::endl;
    std::cout << " - baudrate: " << baudrate << "bps" << std::endl;
    std::cout << " - read/write timeout: " << timeout << "s" << std::endl;
    std::cout << " - slave address: " << slave_address << std::endl;

    const bool connected = driver->connect();
    if (!connected)
    {
      std::cout << "The gripper is not connected" << std::endl;
      return 1;
    }

    std::cout << "The gripper is connected." << std::endl;
    std::cout << "Deactivating the gripper..." << std::endl;
    ;

    driver->deactivate();

    std::cout << "The gripper is deactivated." << std::endl;
    std::cout << "Activating gripper..." << std::endl;
    ;

    driver->activate();

    std::cout << "The gripper is activated." << std::endl;
    std::cout << "Closing the gripper..." << std::endl;

    driver->set_gripper_position(0xFF);
    while (driver->gripper_is_moving())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "Opening the gripper..." << std::endl;
    driver->set_gripper_position(0x00);
    while (driver->gripper_is_moving())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "Half closing the gripper..." << std::endl;
    driver->set_gripper_position(0x80);
    while (driver->gripper_is_moving())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "Opening gripper..." << std::endl;
    driver->set_gripper_position(0x00);
    while (driver->gripper_is_moving())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "Decreasing gripper speed..." << std::endl;
    driver->set_speed(0x0F);

    std::cout << "Closing gripper...\n";
    driver->set_gripper_position(0xFF);
    while (driver->gripper_is_moving())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "Increasing gripper speed..." << std::endl;
    driver->set_speed(0xFF);

    std::cout << "Opening gripper..." << std::endl;
    driver->set_gripper_position(0x00);
    while (driver->gripper_is_moving())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }
  catch (const std::exception& e)
  {
    std::cout << "Failed to communicating with the gripper: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}

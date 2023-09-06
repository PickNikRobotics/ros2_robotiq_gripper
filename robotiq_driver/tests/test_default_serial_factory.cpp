// Copyright (c) 2023 PickNik, Inc.
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

#include <gtest/gtest.h>

#include <robotiq_driver/default_serial_factory.hpp>

#include <mock/mock_serial.hpp>

#include <hardware_interface/hardware_info.hpp>

namespace robotiq_driver::test
{
// This factory will populate the injected mock with data read form the HardwareInfo.
class StubSerialFactory : public DefaultSerialFactory
{
public:
  explicit StubSerialFactory(std::unique_ptr<MockSerial> serial) : serial_{ std::move(serial) }
  {
  }

protected:
  std::unique_ptr<Serial> create_serial() const override
  {
    return std::move(serial_);
  }

private:
  mutable std::unique_ptr<MockSerial> serial_;
};

/**
 * Here we test the serial factory with default parameters.
 */
TEST(TestDefaultSerialFactory, create_with_default_parameters)
{
  hardware_interface::HardwareInfo info;

  auto serial = std::make_unique<MockSerial>();

  // This line is only required when running the test inside the IDE.
  testing::Mock::AllowLeak(serial.get());

  EXPECT_CALL(*serial, set_port("/dev/ttyUSB0"));
  EXPECT_CALL(*serial, set_baudrate(115200));
  EXPECT_CALL(*serial, set_timeout(std::chrono::milliseconds{ 500 }));

  StubSerialFactory serial_factory(std::move(serial));
  auto created_serial = serial_factory.create(info);
}

/**
 * Here we test the serial factory with default parameters.
 */
TEST(TestDefaultSerialFactory, create_with_given_parameters)
{
  hardware_interface::HardwareInfo info;
  info.hardware_parameters.emplace("COM_port", "/dev/ttyUSB1");
  info.hardware_parameters.emplace("baudrate", "9600");
  info.hardware_parameters.emplace("timeout", "0.1");

  auto serial = std::make_unique<MockSerial>();

  // This line is only required when running the test inside the IDE.
  testing::Mock::AllowLeak(serial.get());

  EXPECT_CALL(*serial, set_port("/dev/ttyUSB1"));
  EXPECT_CALL(*serial, set_baudrate(9600));
  EXPECT_CALL(*serial, set_timeout(std::chrono::milliseconds(100)));

  StubSerialFactory serial_factory(std::move(serial));
  auto created_serial = serial_factory.create(info);
}
}  // namespace robotiq_driver::test

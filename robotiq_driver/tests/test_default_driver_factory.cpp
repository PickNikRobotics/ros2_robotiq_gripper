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

#include <robotiq_driver/default_driver_factory.hpp>

#include <mock/mock_driver.hpp>

#include <hardware_interface/hardware_info.hpp>

namespace robotiq_driver::test
{
// This factory will populate the injected mock with data read form the HardwareInfo.
class TestDriverFactory : public DefaultDriverFactory
{
public:
  explicit TestDriverFactory(std::unique_ptr<Driver> driver) : driver_{ std::move(driver) }
  {
  }

protected:
  std::unique_ptr<Driver> create_driver([[maybe_unused]] const hardware_interface::HardwareInfo& info) const override
  {
    return std::move(driver_);
  }

private:
  mutable std::unique_ptr<Driver> driver_;
};

/**
 * Here we test the driver factory with default parameters.
 */
TEST(TestDefaultDriverFactory, create_with_default_parameters)
{
  hardware_interface::HardwareInfo info;

  auto driver = std::make_unique<MockDriver>();

  // This line is only required when running the test inside the IDE.
  testing::Mock::AllowLeak(driver.get());

  EXPECT_CALL(*driver, set_slave_address(0x9));
  EXPECT_CALL(*driver, set_speed(0xFF));
  EXPECT_CALL(*driver, set_force(0xFF));
  EXPECT_CALL(*driver, connect()).Times(0);
  EXPECT_CALL(*driver, disconnect()).Times(0);
  EXPECT_CALL(*driver, activate()).Times(0);
  EXPECT_CALL(*driver, deactivate()).Times(0);

  TestDriverFactory driver_factory{ std::move(driver) };
  auto created_driver = driver_factory.create(info);
}

/**
 * Here we test the driver factory with given parameters.
 */
TEST(TestDefaultDriverFactory, create_with_given_parameters)
{
  hardware_interface::HardwareInfo info;

  info.hardware_parameters.emplace("slave_address", "1");
  info.hardware_parameters.emplace("gripper_speed_multiplier", "0.5");
  info.hardware_parameters.emplace("gripper_force_multiplier", "0.5");

  auto driver = std::make_unique<MockDriver>();

  // This line is only required when running the test inside the IDE.
  testing::Mock::AllowLeak(driver.get());

  EXPECT_CALL(*driver, set_slave_address(0x1));
  EXPECT_CALL(*driver, set_speed(0x7F));
  EXPECT_CALL(*driver, set_force(0x7F));
  EXPECT_CALL(*driver, connect()).Times(0);
  EXPECT_CALL(*driver, disconnect()).Times(0);
  EXPECT_CALL(*driver, activate()).Times(0);
  EXPECT_CALL(*driver, deactivate()).Times(0);

  TestDriverFactory driver_factory{ std::move(driver) };
  auto created_driver = driver_factory.create(info);
}
}  // namespace robotiq_driver::test

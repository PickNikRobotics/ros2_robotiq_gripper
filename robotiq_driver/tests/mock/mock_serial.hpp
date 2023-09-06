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

#pragma once

#include <gmock/gmock.h>

#include <string>
#include <vector>

#include "robotiq_driver/serial.hpp"

namespace robotiq_driver::test
{
class MockSerial : public robotiq_driver::Serial
{
public:
  MOCK_METHOD(void, open, (), (override));
  MOCK_METHOD(bool, is_open, (), (override, const));
  MOCK_METHOD(void, close, (), (override));
  MOCK_METHOD(std::vector<uint8_t>, read, (size_t size), (override));
  MOCK_METHOD(void, write, (const std::vector<uint8_t>& buffer), (override));
  MOCK_METHOD(void, set_port, (const std::string& port), (override));
  MOCK_METHOD(std::string, get_port, (), (override, const));
  MOCK_METHOD(void, set_timeout, (std::chrono::milliseconds timeout), (override));
  MOCK_METHOD(std::chrono::milliseconds, get_timeout, (), (override, const));
  MOCK_METHOD(void, set_baudrate, (uint32_t baudrate), (override));
  MOCK_METHOD(uint32_t, get_baudrate, (), (override, const));
};
}  // namespace robotiq_driver::test

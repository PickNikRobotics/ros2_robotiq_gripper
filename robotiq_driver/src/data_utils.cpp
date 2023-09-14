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

#include <array>
#include <string>
#include <vector>

#include <robotiq_driver/data_utils.hpp>

namespace robotiq_driver::data_utils
{
constexpr std::array<char, 16> vChars = {
  '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
};

std::string to_hex(const std::vector<uint8_t>& bytes)
{
  std::string hex;
  for (auto it = std::begin(bytes); it != std::end(bytes); ++it)
  {
    if (it != bytes.begin())
    {
      hex += " ";
    }
    hex += "";
    uint8_t ch = *it;
    hex += vChars[((ch >> 4) & 0xF)];
    hex += vChars[(ch & 0xF)];
  }

  return hex;
}

std::string to_hex(const std::vector<uint16_t>& bytes)
{
  std::string hex;
  for (auto it = std::begin(bytes); it != std::end(bytes); ++it)
  {
    if (it != bytes.begin())
    {
      hex += " ";
    }
    hex += "";
    uint16_t ch = *it;
    hex += vChars[((ch >> 12) & 0xF)];
    hex += vChars[((ch >> 8) & 0xF)];
    hex += vChars[((ch >> 4) & 0xF)];
    hex += vChars[(ch & 0xF)];
  }

  return hex;
}

std::string to_binary_string(const uint8_t byte)
{
  std::string result = "";
  for (int i = 7; i >= 0; --i)
  {
    result += ((byte >> i) & 1) ? '1' : '0';
  }
  return result;
}

uint8_t get_msb(uint16_t value)
{
  return static_cast<uint8_t>(value >> 8) & 0xFF;
}

uint8_t get_lsb(uint16_t value)
{
  return static_cast<uint8_t>(value & 0xFF);
}

}  // namespace robotiq_driver::data_utils

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

#include <array>
#include <cstdint>
#include <string>
#include <vector>

/**
 * Utility class to convert between commonly used data types.
 */
namespace robotiq_driver::data_utils
{
/**
 * Convert a sequence of uint8_t into a sequence of hex numbers.
 * @param bytes The sequence of bytes.
 * @return A string containing the sequence of hex numbers.
 */
std::string to_hex(const std::vector<uint8_t>& bytes);

/**
 * Convert a sequence of uint16_t into a sequence of hex numbers.
 * @param bytes The sequence of bytes.
 * @return A string containing the sequence of hex numbers.
 */
std::string to_hex(const std::vector<uint16_t>& bytes);

/**
 * Convert a byte to a binary representation for testing purposes.
 * @param byte The byte to decode.
 * @return The binary representation of the given byte.
 */
std::string to_binary_string(const uint8_t byte);

/**
 * Get the Most Significant Byte (MSB) of the given value.
 * @param value A 16-bits value.
 * @return The Most Significant Byte (MSB) of the given value.
 */
uint8_t get_msb(uint16_t value);

/**
 * Get the Least Significant Byte (LSB) of the given value.
 * @param value A 16-bits value.
 * @return The Least Significant Byte (LSB) of the given value.
 */
uint8_t get_lsb(uint16_t value);

}  // namespace robotiq_driver::data_utils
